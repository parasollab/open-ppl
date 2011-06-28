#include "MPStrategy/TogglePRMStrategy.h"
#include "MPProblem/MPRegion.h"
#include "MPStrategy/MPStrategy.h"

bool done;

TogglePRMStrategy::TogglePRMStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool isInherited) :
   MPStrategyMethod(in_Node, in_pProblem), m_CurrentIteration(0){
   //read input
   if(!isInherited)
      ParseXML(in_Node);
}

TogglePRMStrategy::~TogglePRMStrategy(){
}

#include "boost/lambda/lambda.hpp"

void TogglePRMStrategy::ParseXML(XMLNodeReader& in_Node) {
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
      else if(citr->getName() == "col_node_connection_method"){
         string connectMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Node Connection Method"));
         m_ColNodeConnectionLabels.push_back(connectMethod);
         citr->warnUnrequestedAttributes();
      } 
      else if(citr->getName() == "col_component_connection_method"){
         string connectMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Component Connection Method"));
         m_ColComponentConnectionLabels.push_back(connectMethod);
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
      else if(citr->getName()=="dm_method"){
         dm_label =citr->stringXMLParameter(string("Method"),true,string(""),string("Distance Metric"));
         citr->warnUnrequestedAttributes();
      }
      else
         citr->warnUnknownNode();
   }
   
   in_Node.warnUnrequestedAttributes();
  
   //output for debugging
   using boost::lambda::_1;
   cout << "\nTogglePRMStrategy::ParseXML:\n";
   cout << "\tnode_generation_methods: "; 
   for(vector<pair<string, int> >::iterator I = m_NodeGenerationLabels.begin(); I!=m_NodeGenerationLabels.end(); I++){
      cout<<I->first<<"\tNumber:"<<I->second<<" ";
   }
   cout << endl;
   cout << "\tnode_connection_methods: "; for_each(m_NodeConnectionLabels.begin(), m_NodeConnectionLabels.end(), cout << _1 << " "); cout << endl;
   cout << "\tcomponent_connection_methods: "; for_each(m_ComponentConnectionLabels.begin(), m_ComponentConnectionLabels.end(), cout << _1 << " "); cout << endl;
   cout << "\tcol_node_connection_methods: "; for_each(m_ColNodeConnectionLabels.begin(), m_ColNodeConnectionLabels.end(), cout << _1 << " "); cout << endl;
   cout << "\tcol_component_connection_methods: "; for_each(m_ColComponentConnectionLabels.begin(), m_ColComponentConnectionLabels.end(), cout << _1 << " "); cout << endl;
   cout << "\tevaluator_methods: "; for_each(m_EvaluatorLabels.begin(), m_EvaluatorLabels.end(), cout << _1 << " "); cout << endl;
   cout << "\tlp_method: " << m_LPMethod;
   cout << endl;
}

void TogglePRMStrategy::PrintOptions(ostream& out_os) {
   out_os<<"TogglePRMStrategy::PrintOptions()\n";
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
   out_os<<"\nColNodeConnectors\n";
   for(SIT sit=m_ColNodeConnectionLabels.begin(); sit!=m_ColNodeConnectionLabels.end(); sit++){
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
   out_os<<"\nColComponentConnectors\n";
   for(SIT sit=m_ColComponentConnectionLabels.begin(); sit!=m_ColComponentConnectionLabels.end(); sit++){
      out_os<<"\t"<<*sit<<"\tOptions:\n";
      GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetComponentMethod(*sit)->PrintOptions(out_os);
   }
   out_os<<"\nMapEvaluators\n";
   for(SIT sit=m_EvaluatorLabels.begin(); sit!=m_EvaluatorLabels.end(); sit++){
      out_os<<"\t"<<*sit<<"\tOptions:\n";
      GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*sit)->PrintOptions(out_os);
   }
}

void TogglePRMStrategy::Initialize(int in_RegionID){
   cout<<"\nInitializing TogglePRMStrategy::"<<in_RegionID<<endl;

   //seed random number generator
   OBPRM_srand(getSeed());

   //GetMPProblem()->GetValidityChecker()->ToggleValidity();
 
   cout<<"\nEnding Initializing TogglePRMStrategy"<<endl;
}

void TogglePRMStrategy::Run(int in_RegionID){
   cout<<"\nRunning TogglePRMStrategy::"<<in_RegionID<<endl;

   //setup region variables
   MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
   Stat_Class* regionStats = region->GetStatClass();
 
   vector<VID> allNodesVID, allCollisionNodesVID;
   region->GetRoadmap()->m_pRoadmap->GetVerticesVID(allNodesVID);
   vector<VID> thisIterationNodesVID, thisIterationCollisionNodesVID;

   deque<pair<string, VID> > queue;

   MapGenClock.StartClock("Map Generation");
   done=false;
   bool mapPassedEvaluation = false;
   while(!EvaluateMap(in_RegionID)&&!done){
      m_CurrentIteration++;
      cout << "\ngenerating nodes: ";
      GenerateNodes(region, 
            back_insert_iterator<vector<VID> >(allNodesVID), 
            back_insert_iterator<vector<VID> >(thisIterationNodesVID),
            back_insert_iterator<vector<VID> >(allCollisionNodesVID),
            back_insert_iterator<vector<VID> >(thisIterationCollisionNodesVID),
            queue);
      /*GetMPProblem()->GetValidityChecker()->ToggleValidity();
      GenerateNodes(region, 
            back_insert_iterator<vector<VID> >(allCollisionNodesVID),
            back_insert_iterator<vector<VID> >(thisIterationCollisionNodesVID),
            back_insert_iterator<vector<VID> >(allNodesVID),
            back_insert_iterator<vector<VID> >(thisIterationNodesVID));
      GetMPProblem()->GetValidityChecker()->ToggleValidity();
      */
      while(!EvaluateMap(in_RegionID) && queue.size()>0 && !done){
         /*cout<<"size of queue::"<<queue.size()<<endl;
         cout<<"size of allVID::"<<allNodesVID.capacity()<<endl;
         cout<<"size of allColVID::"<<allCollisionNodesVID.capacity()<<endl;
         cout<<"size of thisVID::"<<thisIterationNodesVID.capacity()<<endl;
         cout<<"size of thisColVID::"<<thisIterationCollisionNodesVID.capacity()<<endl;
         cout<<"size of g_free::"<<region->GetRoadmap()->m_pRoadmap->get_num_vertices()<<"\t"<<region->GetRoadmap()->m_pRoadmap->get_num_edges()<<endl;
         cout<<"size of g_obst::"<<region->GetBlockRoadmap()->m_pRoadmap->get_num_vertices()<<"\t"<<region->GetBlockRoadmap()->m_pRoadmap->get_num_edges()<<endl;*/
         pair<string, VID> p = queue.front();
         queue.pop_front();
         string validity = p.first;
         VID vid = p.second;
         cout<<"validity - " << validity<<endl;
         cout<<"vid - "<<vid<<endl;
         if(validity=="valid"){
            Connect(region, p, allNodesVID, allNodesVID, allCollisionNodesVID, queue);
         }
         else if(validity=="invalid"){
            GetMPProblem()->GetValidityChecker()->ToggleValidity();
            Connect(region, p, allCollisionNodesVID, allNodesVID, allCollisionNodesVID, queue);
            GetMPProblem()->GetValidityChecker()->ToggleValidity();
         }
      }
   }
  
   MapGenClock.StopPrintClock();

   cout<<"\nEnd Running TogglePRMStrategy::"<<in_RegionID<<endl;
}

void TogglePRMStrategy::Finalize(int in_RegionID){
   cout<<"\nFinalizing TogglePRMStrategy::"<<in_RegionID<<endl;
   
   //setup region variables
   MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
   Stat_Class* regionStats = region->GetStatClass();
 
   string str;
  
   //output final map
   str = getBaseFilename() + ".map";
   ofstream osMap(str.c_str());
   region->WriteRoadmapForVizmo(osMap);
   osMap.close();

   str = getBaseFilename() + ".block.map";
   ofstream osMap2(str.c_str());
   region->WriteRoadmapForVizmo(osMap2, NULL, true);
   osMap2.close();
  
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

   cout<<"\nEnd Finalizing TogglePRMStrategy"<<endl;
}

void TogglePRMStrategy::Connect(MPRegion<CfgType, WeightType>* region, pair<string, VID> pvid, vector<VID>& allVID, vector<VID>& allNodesVID, vector<VID>&
allCollisionNodesVID, deque<pair<string, VID> >& queue){
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
      vector<VID> nodesVID;
      nodesVID.push_back(pvid.second);
      vector<CfgType> collision, valid;
      if(pvid.first=="valid")
         GetMPProblem()->GetMPStrategy()->GetConnectMap()->ConnectNodes(pConnection,
                                       region->GetRoadmap(), *(region->GetStatClass()),
				       GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                                       GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                                       GetMPProblem()->GetMPStrategy()->addAllEdges,
                                       nodesVID.begin(), nodesVID.end(), 
                                       allVID.begin(), allVID.end(),
                                       back_inserter(collision));
      else
         GetMPProblem()->GetMPStrategy()->GetConnectMap()->ConnectNodes(pConnection,
                                       region->GetBlockRoadmap(), *(region->GetStatClass()),
				       GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                                       GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                                       GetMPProblem()->GetMPStrategy()->addAllEdges,
                                       nodesVID.begin(), nodesVID.end(), 
                                       allVID.begin(), allVID.end(),
                                       back_inserter(collision));
      
      cout<<"\nCollision Nodes from connecting::"<<collision.size()<<endl;
      //filter(region->GetBlockRoadmap()->m_pRoadmap, collision);
      cout<<"Adding "<<collision.size()<<" collision nodes"<<endl;
      typedef vector<CfgType>::iterator CIT;
      int i =0;
      //random_shuffle(collision.begin(), collision.end());
      for(CIT cit=collision.begin(); cit!=collision.end(); ++cit){
         //if(region->GetRoadmap()->m_pRoadmap->get_num_vertices()+
            //region->GetBlockRoadmap()->m_pRoadmap->get_num_vertices()>=1000){done=true;return;}
         if(cit->IsLabel("VALID") && cit->GetLabel("VALID")){
            if(!region->GetRoadmap()->m_pRoadmap->IsVertex(*cit)) {
              VID vid = region->GetRoadmap()->m_pRoadmap->AddVertex(*cit);
              allNodesVID.push_back(vid);
              i++;
              queue.push_front(make_pair("valid", vid));
              //queue.push_back(make_pair("valid", vid));
            }
         }
         //outCollisionNodes mean INVALID then add to block map
         else if(cit->IsLabel("VALID") && !cit->GetLabel("VALID")) {
            if(!region->GetBlockRoadmap()->m_pRoadmap->IsVertex(*cit)) {
              VID vid = region->GetBlockRoadmap()->m_pRoadmap->AddVertex(*cit);
              allCollisionNodesVID.push_back(vid);
              i++;
              queue.push_back(make_pair("invalid", vid));
            }
         }
      }
      cmap.reset();
      cout << region->GetRoadmap()->m_pRoadmap->get_num_edges() << " edges, " 
           << get_cc_count(*(region->GetRoadmap()->m_pRoadmap), cmap) << " connected components"
           << endl;

      cout << "\t";
      NodeConnSubClock.StopPrintClock();
   }
  NodeConnClock.StopPrintClock();
}

bool TogglePRMStrategy::EvaluateMap(int in_RegionID)
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
         if(mapPassedEvaluation){
            //return true;
            cout << "\t  (passed)\n";
         }
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

