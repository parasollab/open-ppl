#include "ProbabilityPRMStrategy.h"
#include "MPCharacterizer.h"
#include "MapEvaluator.h"

ProbabilityPRMStrategy::ProbabilityPRMStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
   BasicPRMStrategy(in_Node, in_pProblem, true){
   //read input
   ParseXML(in_Node);
}

ProbabilityPRMStrategy::~ProbabilityPRMStrategy(){
}

#include "boost/lambda/lambda.hpp"

void ProbabilityPRMStrategy::ParseXML(XMLNodeReader& in_Node) {

   m_NodeGenerationLabels.clear();

   for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr){
      if(citr->getName() == "node_generation_method") {
         string generationMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Node Connection Method"));
         double probPerIteration = citr->numberXMLParameter(string("Probability"), true, double(0), double(0), double(1), string("Number of samples"));
         m_NodeGenerationLabels.push_back(pair<string, double>(generationMethod, probPerIteration));
         citr->warnUnrequestedAttributes();
         
      } 
      else if(citr->getName() == "node_connection_method"){
         string connectMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Node Connection Method"));
         m_NodeConnectionLabels.push_back(connectMethod);
         citr->warnUnrequestedAttributes();
      } 
      else if(citr->getName() == "component_connection_method"){
         string connectMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Component Connection Method"));
         m_ComponentConnectionLabels.push_back(connectMethod);
         citr->warnUnrequestedAttributes();
      } 
      else if(citr->getName() == "evaluation_method"){
         string evalMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Evaluation Method"));
         m_EvaluatorLabels.push_back(evalMethod);
         citr->warnUnrequestedAttributes();
      }
      else if(citr->getName() == "lp_method"){
         m_LPMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Local Planning Method"));
         citr->warnUnrequestedAttributes();
         } 
      else
         citr->warnUnknownNode();
   }
  
   //output for debugging
   double total=0;
   using boost::lambda::_1;
   cout << "\nProbabilityPRMStrategy::ParseXML:\n";
   cout << "\tnode_generation_methods: "; 
   for(vector<pair<string, double> >::iterator I = m_NodeGenerationLabels.begin(); I!=m_NodeGenerationLabels.end(); I++){
      cout<<I->first<<"\tProbability:"<<I->second<<" ";
      total+=I->second;
   }
   if(total!=1.0){cout<<"Probabilities Do Not Add To 1"; exit(1);}
   cout << endl;
   cout << "\tnode_connection_methods: "; for_each(m_NodeConnectionLabels.begin(), m_NodeConnectionLabels.end(), cout << _1 << " "); cout << endl;
   cout << "\tcomponent_connection_methods: "; for_each(m_ComponentConnectionLabels.begin(), m_ComponentConnectionLabels.end(), cout << _1 << " "); cout << endl;
   cout << "\tevaluator_methods: "; for_each(m_EvaluatorLabels.begin(), m_EvaluatorLabels.end(), cout << _1 << " "); cout << endl;
   cout << "\tlp_method: " << m_LPMethod;
   cout << endl;
}

void ProbabilityPRMStrategy::PrintOptions(ostream& out_os) {
   out_os<<"ProbabilityPRMStrategy::PrintOptions()\n";
   out_os<<"\nNodeGenerators\n";
   typedef vector<pair<string,double> >::iterator PIT;
   typedef vector<string>::iterator SIT;
   for(PIT pit=m_NodeGenerationLabels.begin(); pit!=m_NodeGenerationLabels.end(); pit++){
      out_os<<"\t"<<pit->first<<"\tProbability of Choosing Sampler:"<<pit->second<<"\tOptions:\n";
      GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(pit->first)->PrintOptions(out_os);
   }
   out_os<<"\nNodeConnectors\n";
   for(SIT sit=m_NodeConnectionLabels.begin(); sit!=m_NodeConnectionLabels.end(); sit++){
      out_os<<"\t"<<*sit<<"\tOptions:\n";
      GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetNodeMethod(*sit)->PrintOptions(out_os);
   }
   out_os<<"\nLocalPlanner\n";
   out_os<<"\t"<<m_LPMethod<<"\tOptions:\n";
   GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_LPMethod)->PrintOptions(out_os);
   out_os<<"\nComponentConnectors\n";
   for(SIT sit=m_ComponentConnectionLabels.begin(); sit!=m_ComponentConnectionLabels.end(); sit++){
      out_os<<"\t"<<*sit<<"\tOptions:\n";
      GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetComponentMethod(*sit)->PrintOptions(out_os);
   }
   out_os<<"\nMapEvaluators\n";
   for(SIT sit=m_EvaluatorLabels.begin(); sit!=m_EvaluatorLabels.end(); sit++){
      out_os<<"\t"<<*sit<<"\tOptions:\n";
      GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*sit)->PrintOptions(out_os);
   }
}

void ProbabilityPRMStrategy::operator()(int in_RegionID)
{
   cout << "\n\nBeginning ProbabilityPRMStratgy::operator(" << in_RegionID << ")\n";
  
   //seed random number generator
   OBPRM_srand(getSeed()); 
  
   //setup region variables
   MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
   Stat_Class* regionStats = region->GetStatClass();
  
   vector<VID> allNodesVID;
   region->GetRoadmap()->m_pRoadmap->GetVerticesVID(allNodesVID);
  
   Clock_Class MapGenClock;
   MapGenClock.StartClock("Map Generation");
  
   bool mapPassedEvaluation = false;
   while(!mapPassedEvaluation){
      m_CurrentIteration++;
      vector<VID> thisIterationNodesVID;
      cout << "\ngenerating nodes: ";
      GenerateNodes(region, 
                    back_insert_iterator<vector<VID> >(allNodesVID), 
                    back_insert_iterator<vector<VID> >(thisIterationNodesVID));
      cout << "\nconnecting nodes: ";
      ConnectNodes(region, allNodesVID, thisIterationNodesVID);
      ConnectComponents(region);
      cout << "\nevaluating roadmap: ";
      mapPassedEvaluation = EvaluateMap(in_RegionID);
   }
  
   MapGenClock.StopPrintClock();
  
   string str;
  
   //output final map
   str = getBaseFilename() + ".map";
   ofstream osMap(str.c_str());
   region->WriteRoadmapForVizmo(osMap);
   osMap.close();
  
   //output stats
   str = getBaseFilename() + ".stat";
   ofstream  osStat(str.c_str());
   streambuf* sbuf = cout.rdbuf(); // to be restored later
   cout.rdbuf(osStat.rdbuf());   // redirect destination of std::cout
   cout << "NodeGen+Connection Stats" << endl;
   regionStats->PrintAllStats(region->GetRoadmap());
   MapGenClock.PrintClock();
   //regionStats->PrintFeatures();
   cout.rdbuf(sbuf);  // restore original stream buffer
   osStat.close();

   cout << "Ending ProbabilityPRMStratgy::operator(" << in_RegionID << ")\n\n";
}

string ProbabilityPRMStrategy::PickNextSampler(){
   double rand = OBPRM_drand();
   double total = 0;
   int index=-1;
   typedef vector<pair<string, double> >::iterator GIT;
   for(GIT git = m_NodeGenerationLabels.begin(); git!=m_NodeGenerationLabels.end(); git++){
      if(rand<=total){
         break;
      }
      else{
         total+=git->second;
         index++;
      }
   }
   cout<<"Choosing "<<m_NodeGenerationLabels[index].first<<" as the next node generator"<<endl;
   return m_NodeGenerationLabels[index].first;
}

