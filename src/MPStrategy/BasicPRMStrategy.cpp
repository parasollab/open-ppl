#include "MPStrategy/BasicPRMStrategy.h"
#include "MPProblem/MPRegion.h"
#include "MPStrategy/MPStrategy.h"

BasicPRMStrategy::BasicPRMStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool isInherited) :
   MPStrategyMethod(in_Node, in_pProblem), m_CurrentIteration(0){
   //read input
   if(!isInherited)
      ParseXML(in_Node);
}

BasicPRMStrategy::~BasicPRMStrategy(){
}

#include "boost/lambda/lambda.hpp"

void BasicPRMStrategy::ParseXML(XMLNodeReader& in_Node) {
   for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr){
      if(citr->getName() == "node_generation_method") {
         string generationMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Node Connection Method"));
         int numPerIteration = citr->numberXMLParameter(string("Number"), true, int(1), int(0), MAX_INT, string("Number of samples"));
         m_NodeGenerationLabels.push_back(pair<string, int>(generationMethod, numPerIteration));
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
      } else if(citr->getName()=="dm_method"){
         dm_label =citr->stringXMLParameter(string("Method"),true,string(""),string("Distance Metric"));
         citr->warnUnrequestedAttributes();
      }
      else
         citr->warnUnknownNode();
   }
  
   //output for debugging
   using boost::lambda::_1;
   cout << "\nBasicPRMStrategy::ParseXML:\n";
   cout << "\tnode_generation_methods: "; 
   for(vector<pair<string, int> >::iterator I = m_NodeGenerationLabels.begin(); I!=m_NodeGenerationLabels.end(); I++){
      cout<<I->first<<"\tNumber:"<<I->second<<" ";
   }
   cout << endl;
   cout << "\tnode_connection_methods: "; for_each(m_NodeConnectionLabels.begin(), m_NodeConnectionLabels.end(), cout << _1 << " "); cout << endl;
   cout << "\tcomponent_connection_methods: "; for_each(m_ComponentConnectionLabels.begin(), m_ComponentConnectionLabels.end(), cout << _1 << " "); cout << endl;
   cout << "\tevaluator_methods: "; for_each(m_EvaluatorLabels.begin(), m_EvaluatorLabels.end(), cout << _1 << " "); cout << endl;
   cout << "\tlp_method: " << m_LPMethod;
   cout << endl;
}

void BasicPRMStrategy::PrintOptions(ostream& out_os) {
   out_os<<"BasicPRMStrategy::PrintOptions()\n";
   out_os<<"\nNodeGenerators\n";
   typedef vector<pair<string,int> >::iterator PIT;
   typedef vector<string>::iterator SIT;
   for(PIT pit=m_NodeGenerationLabels.begin(); pit!=m_NodeGenerationLabels.end(); pit++){
      out_os<<"\t"<<pit->first<<"\tNumber of Samples Per Iteration:"<<pit->second<<"\tOptions:\n";
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

void BasicPRMStrategy::Initialize(int in_RegionID){
   cout<<"\nInitializing BasicPRMStrategy::"<<in_RegionID<<endl;

   //seed random number generator
   OBPRM_srand(getSeed()); 
 
   cout<<"\nEnding Initializing BasicPRMStrategy"<<endl;
}

void BasicPRMStrategy::Run(int in_RegionID){
   cout<<"\nRunning BasicPRMStrategy::"<<in_RegionID<<endl;

   //setup region variables
   MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
   Stat_Class* regionStats = region->GetStatClass();
 
   vector<VID> allNodesVID;
   region->GetRoadmap()->m_pRoadmap->GetVerticesVID(allNodesVID);
  
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

   cout<<"\nEnd Running BasicPRMStrategy::"<<in_RegionID<<endl;
}

void BasicPRMStrategy::Finalize(int in_RegionID){
   cout<<"\nFinalizing BasicPRMStrategy::"<<in_RegionID<<endl;
   
   //setup region variables
   MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
   Stat_Class* regionStats = region->GetStatClass();
 
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

   cout<<"\nEnd Finalizing BasicPRMStrategy"<<endl;
}

void BasicPRMStrategy::ConnectNodes(MPRegion<CfgType, WeightType>* region, 
                                 vector<VID>& allNodesVID, vector<VID>& thisIterationNodesVID)
{
   Clock_Class NodeConnClock;
   stringstream clockName; clockName << "Iteration " << m_CurrentIteration << ", Node Connection";
   NodeConnClock.StartClock(clockName.str().c_str());
    stapl::vector_property_map< GRAPH,size_t > cmap;
  
   for(vector<string>::iterator I = m_NodeConnectionLabels.begin(); 
       I != m_NodeConnectionLabels.end(); ++I){
    
      ConnectMap<CfgType, WeightType>::NodeConnectionPointer pConnection;
      pConnection = GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetNodeMethod(*I);    
    
      Clock_Class NodeConnSubClock;
      stringstream connectorClockName; connectorClockName << "Iteration " << m_CurrentIteration << ", " << pConnection->GetName();
      NodeConnSubClock.StartClock(connectorClockName.str().c_str());
    
      cout << "\n\t";
      vector<VID> nodesVID(thisIterationNodesVID.begin(), thisIterationNodesVID.end());
      vector<CfgType> collision;
      GetMPProblem()->GetMPStrategy()->
         GetConnectMap()->ConnectNodes(pConnection,
                                       region->GetRoadmap(), *(region->GetStatClass()),
				       GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                                       GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                                       GetMPProblem()->GetMPStrategy()->addAllEdges,
                                       nodesVID.begin(), nodesVID.end(), 
                                       allNodesVID.begin(), allNodesVID.end(),
                                       back_inserter(collision));
      cmap.reset();
      cout << region->GetRoadmap()->m_pRoadmap->get_num_edges() << " edges, " 
           << get_cc_count(*(region->GetRoadmap()->m_pRoadmap), cmap) << " connected components"
           << endl;

      cout << "\t";
      NodeConnSubClock.StopPrintClock();
   }
   NodeConnClock.StopPrintClock();
}

void BasicPRMStrategy::ConnectComponents(MPRegion<CfgType, WeightType>* region)
{
   Clock_Class ComponentConnClock;
   stringstream clockName; clockName << "Iteration " << m_CurrentIteration << ", Component Connection";
   ComponentConnClock.StartClock(clockName.str().c_str());
    stapl::vector_property_map< GRAPH,size_t > cmap;
 
   for(vector<string>::iterator I = m_ComponentConnectionLabels.begin(); 
       I != m_ComponentConnectionLabels.end(); ++I){
      ConnectMap<CfgType, WeightType>::ComponentConnectionPointer pConnection;
      pConnection = GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetComponentMethod(*I);
    
      Clock_Class ComponentConnSubClock;
      stringstream connectorClockName; connectorClockName << "Iteration " << m_CurrentIteration << ", " << pConnection->GetName();
      ComponentConnSubClock.StartClock(connectorClockName.str().c_str());
    
      cout << "\n\t";
      vector<CfgType> collision;
      GetMPProblem()->GetMPStrategy()->
         GetConnectMap()->ConnectComponents(pConnection,
                                            region->GetRoadmap(), 
                                            *(region->GetStatClass()),
                                            GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                                            GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                                            GetMPProblem()->GetMPStrategy()->addAllEdges,
                                            back_inserter(collision));
      
      cmap.reset();
      cout << region->GetRoadmap()->m_pRoadmap->get_num_edges() << " edges, " 
           << get_cc_count(*(region->GetRoadmap()->m_pRoadmap), cmap) << " connected components"
           << endl;
    
      cout << "\t";
      ComponentConnSubClock.StopPrintClock();
   }
   ComponentConnClock.StopPrintClock();
}


bool BasicPRMStrategy::EvaluateMap(int in_RegionID)
{
   bool mapPassedEvaluation = false;
   if(!m_EvaluatorLabels.empty()){
      Clock_Class EvalClock;
      stringstream clockName; clockName << "Iteration " << m_CurrentIteration << ", Map Evaluation"; 
      EvalClock.StartClock(clockName.str().c_str());
      
      mapPassedEvaluation = true;
      for(vector<string>::iterator I = m_EvaluatorLabels.begin(); 
          I != m_EvaluatorLabels.end(); ++I){
         MapEvaluator<CfgType, WeightType>::conditional_type pEvaluator;
         pEvaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*I);
         Clock_Class EvalSubClock;
         stringstream evaluatorClockName; evaluatorClockName << "Iteration " << m_CurrentIteration << ", " << pEvaluator->GetName();
         EvalSubClock.StartClock(evaluatorClockName.str().c_str());
         
         cout << "\n\t";
         mapPassedEvaluation = pEvaluator->operator()(in_RegionID);
         
         cout << "\t";
         EvalSubClock.StopPrintClock();
         if(mapPassedEvaluation)
            cout << "\t  (passed)\n";
         else
            cout << "\t  (failed)\n";
         if(!mapPassedEvaluation)
            break;
      }
      EvalClock.StopPrintClock();
   }
   else{mapPassedEvaluation=true;}//avoid the infinite loop
   return mapPassedEvaluation;
}

