#include "BasicPRMStrategy.h"
#include "MPRegion.h"
#include "MPStrategy.h"
#include "MapEvaluator.h"
#include "boost/lambda/lambda.hpp"

BasicPRMStrategy::BasicPRMStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
   MPStrategyMethod(in_Node, in_pProblem), m_CurrentIteration(0), useProbability(false){
   //read input
   ParseXML(in_Node);
}

BasicPRMStrategy::~BasicPRMStrategy(){
}

void BasicPRMStrategy::ParseXML(XMLNodeReader& in_Node) {
   for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr){
      if(citr->getName() == "node_generation_method") {
         string generationMethod = citr->stringXMLParameter("Method", true, "", "Node Connection Method");
         int numPerIteration = citr->numberXMLParameter("Number", true, 1, 0, MAX_INT, "Number of samples");
         int attemptsPerIteration = citr->numberXMLParameter("Attempts", false, 1, 0, MAX_INT, "Number of attempts per sample");
         double probPerIteration = 0;
         if(numPerIteration == 0){
          probPerIteration = citr->numberXMLParameter("Probability", true, 0.0, 0.0, 1.0, "Number of samples");
          useProbability = true;
         }
         m_NodeGenerationLabels[generationMethod] = make_pair(numPerIteration, attemptsPerIteration);
         m_ProbGenerationLabels[generationMethod] = make_pair(probPerIteration, attemptsPerIteration);
         citr->warnUnrequestedAttributes();
      } 
      else if(citr->getName() == "node_connection_method"){
         string connectMethod = citr->stringXMLParameter("Method", true, "", "Node Connection Method");
         m_NodeConnectionLabels.push_back(connectMethod);
         citr->warnUnrequestedAttributes();
      } 
      else if(citr->getName() == "component_connection_method"){
         string connectMethod = citr->stringXMLParameter("Method", true, "", "Component Connection Method");
         m_ComponentConnectionLabels.push_back(connectMethod);
         citr->warnUnrequestedAttributes();
      } 
      else if(citr->getName() == "evaluation_method"){
         string evalMethod = citr->stringXMLParameter("Method", true, "", "Evaluation Method");
         m_EvaluatorLabels.push_back(evalMethod);
         citr->warnUnrequestedAttributes();
      }
      else if(citr->getName()=="vc_method"){
         vcMethod =citr->stringXMLParameter("Method",true,"","ValidityCheckerMethod");
         citr->warnUnrequestedAttributes();
      }
      else
         citr->warnUnknownNode();
   }
  
   //output for debugging
   double total=0.0;
   using boost::lambda::_1;
   cout << "\nBasicPRMStrategy::ParseXML:\n";
   cout << "\tnode_generation_methods: \n"; 
   if(!useProbability){
     for(map<string, pair<int, int> >::iterator I = m_NodeGenerationLabels.begin(); I!=m_NodeGenerationLabels.end(); I++){
       cout<<"\t\t"<<I->first<<"\tNumber:"<<I->second.first<<"\tAttempts:"<<I->second.second<<endl;
     }
     total=1.0;
   }
   else{
     for(map<string, pair<double, int> >::iterator I = m_ProbGenerationLabels.begin(); I!=m_ProbGenerationLabels.end(); I++){
       cout<<"\t\t"<<I->first<<"\tProbability:"<<I->second.first<<"\tAttempts:"<<I->second.second<<endl;
       total+=I->second.first;
     }
   }
   if(total<0.999 || total>1.001){cout<<"Probabilities Do Not Add To 1, total is "<<total<<endl; exit(1);}
   cout << endl;
   cout << "\tnode_connection_methods: "; for_each(m_NodeConnectionLabels.begin(), m_NodeConnectionLabels.end(), cout << _1 << " "); cout << endl;
   cout << "\tcomponent_connection_methods: "; for_each(m_ComponentConnectionLabels.begin(), m_ComponentConnectionLabels.end(), cout << _1 << " "); cout << endl;
   cout << "\tevaluator_methods: "; for_each(m_EvaluatorLabels.begin(), m_EvaluatorLabels.end(), cout << _1 << " "); cout << endl;
   cout << endl;
}

void BasicPRMStrategy::PrintOptions(ostream& out_os) {
   out_os<<"BasicPRMStrategy::PrintOptions()\n";
   out_os<<"\nNodeGenerators\n";
   typedef vector<string>::iterator SIT;
   if(!useProbability){
     typedef map<string, pair<int,int> >::iterator PIT;
     for(PIT pit=m_NodeGenerationLabels.begin(); pit!=m_NodeGenerationLabels.end(); pit++){
       out_os<<"\t"<<pit->first<<"\tNumber:"<<pit->second.first<<"\tAttempts:"<<pit->second.second<<"\tOptions:\n";
       GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(pit->first)->PrintOptions(out_os);
     }
   }
   else{
     typedef map<string, pair<double,int> >::iterator PIT;
     for(PIT pit=m_ProbGenerationLabels.begin(); pit!=m_ProbGenerationLabels.end(); pit++){
       out_os<<"\t"<<pit->first<<"\tProbability:"<<pit->second.first<<"\tAttempts:"<<pit->second.second<<"\tOptions:\n";
       GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(pit->first)->PrintOptions(out_os);
     }
   }
   out_os<<"\nNodeConnectors\n";
   for(SIT sit=m_NodeConnectionLabels.begin(); sit!=m_NodeConnectionLabels.end(); sit++){
      out_os<<"\t"<<*sit<<"\tOptions:\n";
      GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetNodeMethod(*sit)->PrintOptions(out_os);
   }
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
      GetMPProblem()->GetMPStrategy()->
         GetConnectMap()->ConnectNodes(pConnection,
                                       region->GetRoadmap(), *(region->GetStatClass()),
				       GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                                       GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                                       GetMPProblem()->GetMPStrategy()->addAllEdges,
                                       nodesVID.begin(), nodesVID.end(), 
                                       allNodesVID.begin(), allNodesVID.end());
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
      GetMPProblem()->GetMPStrategy()->
         GetConnectMap()->ConnectComponents(pConnection,
                                            region->GetRoadmap(), 
                                            *(region->GetStatClass()),
                                            GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                                            GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                                            GetMPProblem()->GetMPStrategy()->addAllEdges);
      
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

string BasicPRMStrategy::PickNextSampler(){
   double rand = OBPRM_drand();
   double total = 1.0;
   string index = "";
   typedef map<string, pair<double,int> >::iterator GIT;
   for(GIT git = m_ProbGenerationLabels.begin(); git!=m_ProbGenerationLabels.end(); git++){
      if(rand>=total){
         break;
      }
      else{
        index = git->first;
         total-=git->second.first;
      }
   }
   cout<<"Choosing "<<index<<" as the next node generator"<<endl;
   return index;
}

template <typename OutputIterator>
void BasicPRMStrategy::GenerateNodes(MPRegion<CfgType, WeightType>* region, 
    OutputIterator allOut, OutputIterator thisIterationOut){
  Clock_Class NodeGenClock;
  CDInfo cdInfo;
  Stat_Class * pStatClass = region->GetStatClass();
  stringstream clockName; 
  clockName << "Iteration " << m_CurrentIteration << ", Node Generation"; 
  NodeGenClock.StartClock(clockName.str().c_str());
  string Callee("BasicPRMStrategy::GenerateNodes");

  typedef map<string, pair<int, int> >::iterator GIT;
  vector<CfgType> outNodes;
  if(!useProbability){
    for(GIT git = m_NodeGenerationLabels.begin(); git != m_NodeGenerationLabels.end(); ++git){
      Sampler<CfgType>::SamplerPointer pNodeGenerator = GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(git->first);
      pNodeGenerator->print(cout);
      vector<CfgType> inNodes(git->second.first);

      //generate nodes for this node generator method
      Clock_Class NodeGenSubClock;
      stringstream generatorClockName; 
      generatorClockName << "Iteration " << m_CurrentIteration << ", " << git->first;
      NodeGenSubClock.StartClock(generatorClockName.str().c_str());

      cout << "\n\t";

      do{
        pNodeGenerator->Sample(GetMPProblem()->GetEnvironment(),*pStatClass,
            inNodes.begin(),inNodes.end(),git->second.second, back_inserter(outNodes));
      }while(outNodes.size()<=0&&m_CurrentIteration==1);

      cout << region->GetRoadmap()->m_pRoadmap->get_num_vertices() << " vertices " << endl;
      cout << "\n\t";
      NodeGenSubClock.StopPrintClock();

    }
  }
  else{
    string NextNodeGen = PickNextSampler();
    Sampler<CfgType>::SamplerPointer pNodeGenerator; 
    pNodeGenerator = GetMPProblem()->GetMPStrategy()->GetSampler()->GetSamplingMethod(NextNodeGen);
    pNodeGenerator->print(cout);
    vector<CfgType> inNodes(1);

    //generate nodes for this node generator method
    Clock_Class NodeGenSubClock;
    stringstream generatorClockName; 
    generatorClockName << "Iteration " << m_CurrentIteration << ", " << NextNodeGen;
    NodeGenSubClock.StartClock(generatorClockName.str().c_str());

    cout << "\n\t";
    pNodeGenerator->Sample(GetMPProblem()->GetEnvironment(),*pStatClass,inNodes.begin(),inNodes.end(),m_ProbGenerationLabels[NextNodeGen].second, back_inserter(outNodes));

    cout << region->GetRoadmap()->m_pRoadmap->get_num_vertices() << " vertices " << endl;

    cout << "\n\t";
    NodeGenSubClock.StopPrintClock();
  }
  //add valid nodes to roadmap
  typedef vector<CfgType>::iterator CIT;
  for(CIT cit=outNodes.begin(); cit!=outNodes.end(); ++cit){
    if(!(*cit).IsLabel("VALID")){
      !(GetMPProblem()->GetValidityChecker()->IsValid(GetMPProblem()->GetValidityChecker()->GetVCMethod(vcMethod), *cit, GetMPProblem()->GetEnvironment(), *(region->GetStatClass()), cdInfo, true, &Callee));
    }
    if((*cit).IsLabel("VALID") && ((*cit).GetLabel("VALID"))) {
      if(!region->GetRoadmap()->m_pRoadmap->IsVertex(*cit)) {
        VID vid = region->GetRoadmap()->m_pRoadmap->AddVertex(*cit);
        //store value and increment iterator
        *thisIterationOut++ = vid;
        *allOut++ = vid;
      }
    }
  }
  NodeGenClock.StopPrintClock();
}
