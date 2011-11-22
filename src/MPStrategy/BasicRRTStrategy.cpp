/**
 * BasicRRTStrategy.cpp
 *
 * Description: Main RRT Strategy, contains RRT method used in RRTconnect
 *
 * Author: Kasra Manavi
 * Last Edited: 04/08/2011
 */

#include "MPStrategy/BasicRRTStrategy.h"
#include "MPProblem/MPRegion.h"
#include "MPStrategy/MPStrategy.h"
#include "Utilities/MPUtils.h"

BasicRRTStrategy::BasicRRTStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool isInherited) :
  MPStrategyMethod(in_Node, in_pProblem), m_CurrentIteration(0){
  if(!isInherited)
    ParseXML(in_Node);
}

BasicRRTStrategy::~BasicRRTStrategy(){ }

#include "boost/lambda/lambda.hpp"

void BasicRRTStrategy::ParseXML(XMLNodeReader& in_Node) {
  for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr){
    if(citr->getName() == "node_generation_method") {
      string generationMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Node Connection Method"));
      int numPerIteration = citr->numberXMLParameter(string("Number"), true, int(1), int(0), MAX_INT, string("Number of samples"));
      m_NodeGenerationLabels.push_back(pair<string, int>(generationMethod, numPerIteration));
      citr->warnUnrequestedAttributes();
    } else if(citr->getName() == "node_connection_method"){
      string connectMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Node Connection Method"));
      m_NodeConnectionLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    } else if(citr->getName() == "component_connection_method"){
      string connectMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Component Connection Method"));
      m_ComponentConnectionLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    } else if(citr->getName() == "evaluation_method"){
      string evalMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Evaluation Method"));
      m_EvaluatorLabels.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    } else if(citr->getName() == "lp_method"){
      m_LPMethod = citr->stringXMLParameter(string("Method"), true, string(""), string("Local Planning Method"));
      citr->warnUnrequestedAttributes();
    } else if(citr->getName()=="dm_method"){
      dm_label = citr->stringXMLParameter(string("Method"),true,string(""),string("Distance Metric"));
      citr->warnUnrequestedAttributes();
    } else if(citr->getName()=="neighborhood_finder") {
      nf_label = citr->stringXMLParameter(string("nf"), true, string(""), string("Neighborhood Finder"));
      citr->warnUnrequestedAttributes();
    } else if(citr->getName()=="validity_checker") {
      strVcmethod = citr->stringXMLParameter(string("vcm"), true, string(""), string("Validity Test Method"));
      citr->warnUnrequestedAttributes();
    } else if(citr->getName()=="rrt_params") {
      delta       = citr->numberXMLParameter(string("delta"), false, 0.05, 0.0, 1.0, string("Delta Distance"));
      minDist     = citr->numberXMLParameter(string("minDist"), false, 0.00, 0.0, 1.0, string("Minimum Distance"));
      obsDist     = citr->numberXMLParameter(string("obsDist"), false, 0.00, 0.0, 1.0, string("Distance from Obstacles"));
      roots       = citr->numberXMLParameter(string("numRoots"), false, 1, 1, 100, string("Number of Roots"));
      growthFocus = citr->numberXMLParameter(string("growthFocus"), false, 0.00, 0.0, 1.0, string("#GeneratedTowardsGoal/#Generated"));
      citr->warnUnrequestedAttributes();
    } else
      citr->warnUnknownNode();
  }
  
  using boost::lambda::_1;
  cout << "\nBasicRRTStrategy::ParseXML:\n";
  cout << "\tnode_generation_methods: "; 
  for(vector<pair<string, int> >::iterator I = m_NodeGenerationLabels.begin(); I!=m_NodeGenerationLabels.end(); I++)
    cout<<I->first<<"\tNumber:"<<I->second<<" ";
  cout << endl;
  cout << "\tdelta:      " << delta << endl;
  cout << "\tminDist:    " << minDist << endl;
  cout << "\tobsDist:     " << obsDist << endl;
  cout << "\tnumRoots:    " << roots << endl;
  cout << "\tgrowthFocus: " << growthFocus << endl;
  cout << "\tnode_connection_methods: "; for_each(m_NodeConnectionLabels.begin(), m_NodeConnectionLabels.end(), cout << _1 << " "); cout << endl;
  cout << "\tcomponent_connection_methods: "; for_each(m_ComponentConnectionLabels.begin(), m_ComponentConnectionLabels.end(), cout << _1 << " "); cout << endl;
  cout << "\tevaluator_methods: "; for_each(m_EvaluatorLabels.begin(), m_EvaluatorLabels.end(), cout << _1 << " "); cout << endl;
  cout << "\tlp_method: " << m_LPMethod << endl;
  cout << "\tdm_method: " << dm_label << endl;
  cout << "\tnf_method: " << nf_label << endl;
  cout << "\tstrVcmethod = " << strVcmethod << endl;  
}

void BasicRRTStrategy::PrintOptions(ostream& out_os) {
  out_os<<"BasicRRTStrategy::PrintOptions()\n";
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
  GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetLocalPlannerMethod(m_LPMethod)->PrintOptions(out_os);
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

void BasicRRTStrategy::Initialize(int in_RegionID){
   cout<<"\nInitializing BasicRRTStrategy::"<<in_RegionID<<endl;
   OBPRM_srand(getSeed()); 
   cout<<"\nEnding Initializing BasicRRTStrategy"<<endl;
}

void BasicRRTStrategy::Run(int in_RegionID) {

  cout << "\nRunning BasicRRTStrategy::" << in_RegionID << endl;

  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
  /*
  // Goal setup
  CfgType goal1;
  goal1.SetSingleParam(0,48);
  goal1.SetSingleParam(1,48);
  goal1.SetSingleParam(2,48);
  cout << "GOAL: "<< goal1 << endl;
  CfgType goal2;
  goal2.SetSingleParam(0,-48);
  goal2.SetSingleParam(1,-48);
  goal2.SetSingleParam(2,-48);
  cout << "GOAL: "<< goal2 << endl;
  */
  vector<CfgType> goals;
  //goals.push_back(goal1);
  //goals.push_back(goal2);

  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation){
    // Actual RRT construction
    RRT(in_RegionID, goals);
    ConnectComponents(region);
    mapPassedEvaluation = EvaluateMap(in_RegionID);
  }
  
  cout<<"\nEnd Running BasicRRTStrategy::" << in_RegionID << endl;  

}

// MAIN RRT METHOD
void BasicRRTStrategy::RRT(int in_RegionID, vector<CfgType> RRTQueue) {
  
  // Setup MP Variables
  MPRegion<CfgType,WeightType>*      region = GetMPProblem()->GetMPRegion(in_RegionID);
  Stat_Class*                        regionStats = region->GetStatClass();
  Environment*                       env = region->GetRoadmap()->GetEnvironment();
  shared_ptr <DistanceMetricMethod>  _dm = GetMPProblem()->GetDistanceMetric()->GetDMMethod(dm_label);
  LocalPlanners<CfgType,WeightType>* lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
  ValidityChecker<CfgType>*          vc = GetMPProblem()->GetValidityChecker();
  LPOutput<CfgType,WeightType>       lpOutput;
  CDInfo                             cdInfo;
  cdInfo.ret_all_info = true;
  string callee("BasicRRTStrategy::RRT");
  
  // Setup RRT Variables
  CfgType tmp, dir;
  bool checkCollision=false, savePath=false, saveFailed=false, mapPassed=false;
  vector<bool> found;
  for (size_t j=0; j<RRTQueue.size(); ++j)
    found.push_back(false);
  
  MapGenClock.StartClock("RRT Generation");
  
  // Add root vertex/vertices
  for (int i=0; i<roots; ++i) {
    tmp.GetRandomCfg(env);
    CfgType root = CfgType(tmp);
    if (root.InBoundingBox(env)
	&& vc->IsValid(vc->GetVCMethod(strVcmethod), root, env, *regionStats, cdInfo, true, &callee))
      region->GetRoadmap()->m_pRoadmap->AddVertex(root);
    else 
      --i;
  }
  
  // For the number of iterations
  for (int h=0; h<m_iterations; ++h) {
    // Main loop for generation
    for (int i=0; i<num_nodes; ++i) {
      
      // Check if done
      bool done = true;
      if (found.size() == 0)
	done = false;
      for (size_t j=0; j<found.size(); ++j)
	done = (done && found[j]);
      if (done){
	cout << "RRT FOUND ALL GOALS... We can call it doneziez" << endl;
	break;
      }
      
      
      // Determine direction, make sure goal not found
      tmp.GetRandomCfg(env);
      CfgType randomCfg = CfgType(tmp);     
      double randomRatio = (double)(OBPRM_lrand()%1000)/1000;
      bool usingQueue = false;
      int goalNum;
      
      if (RRTQueue.size() == 0)
	goalNum = -1;
      else
	goalNum = i%RRTQueue.size();
      
      if ( (RRTQueue.size() > 0) && (randomRatio<growthFocus)) {
	if ( found[goalNum] ) {
	  for (size_t j=0; j<found.size(); j++) {
	    if ( !(found[j]))
	      goalNum = j;
	  }
	}
	dir = RRTQueue[goalNum];
	usingQueue = true;
      } else
	dir = randomCfg;
      
      // Find closest Cfg in map
      vector< VID > kClosest;
      vector< VID >::iterator startKIterator;
      vector<CfgType> cfgs;
      vector<CfgType>::iterator startCIterator;
      
      GetMPProblem()->GetNeighborhoodFinder()->KClosest( GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(nf_label),
							 region->GetRoadmap(), dir, 1, back_inserter(kClosest));     
      CfgType nearest = region->GetRoadmap()->m_pRoadmap->find_vertex(kClosest[0])->property();
      CfgType new_cfg;
      
      if(!RRTExpand(GetMPProblem(), in_RegionID, strVcmethod, dm_label, nearest, dir, new_cfg, delta, obsDist)) {
        continue;
      }

      // Check if goal has been found
      if (usingQueue && _dm->Distance(env, dir, new_cfg) < (delta)/100) {
	new_cfg = dir;
	found[goalNum] = true;
      }
     


      // If good to go, add to roadmap
      double positionRes    = env->GetPositionRes();
      double orientationRes = env->GetOrientationRes();
      CfgType collNode;
      if (new_cfg.InBoundingBox(env)
	  && vc->IsValid(vc->GetVCMethod(strVcmethod), new_cfg, env, *regionStats, cdInfo, true, &callee)
	  && (_dm->Distance(env, new_cfg, nearest) >= minDist)
	  && lp->GetLocalPlannerMethod(m_LPMethod)->IsConnected(env, *regionStats, _dm, nearest, new_cfg, collNode, &lpOutput,
						    positionRes, orientationRes, checkCollision, savePath, saveFailed)) {
	region->GetRoadmap()->m_pRoadmap->AddVertex(new_cfg);
	region->GetRoadmap()->m_pRoadmap->AddEdge(nearest, new_cfg, lpOutput.edge);
      } else 
	--i;
    }

    // Evalutate
    mapPassed = EvaluateMap(in_RegionID);
    if (mapPassed)
      cout << "Map Evaluator Passed... Iteration = " << h << endl;
  }

  MapGenClock.StopPrintClock();
  cout<<"\nEnd Running BasicRRTStrategy::"<<in_RegionID;
}

void BasicRRTStrategy::Finalize(int in_RegionID) {

  cout<<"\nFinalizing BasicRRTStrategy::"<<in_RegionID<<endl;
  
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
  
  cout<<"\nEnd Finalizing BasicRRTStrategy"<<endl;
}

void BasicRRTStrategy::ConnectComponents(MPRegion<CfgType, WeightType>* region) {
  
  Clock_Class ComponentConnClock;
  stringstream clockName; clockName << "Iteration " << m_CurrentIteration << ", Component Connection";
  ComponentConnClock.StartClock(clockName.str().c_str());
  stapl::vector_property_map< GRAPH,size_t > cmap;
  
  for (vector<string>::iterator I = m_ComponentConnectionLabels.begin(); 
       I != m_ComponentConnectionLabels.end(); ++I) {
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


bool BasicRRTStrategy::EvaluateMap(int in_RegionID) {

  bool mapPassedEvaluation = false;
  if ( !m_EvaluatorLabels.empty() ) {

    Clock_Class EvalClock;
    stringstream clockName; clockName << "Iteration " << m_CurrentIteration << ", Map Evaluation"; 
    EvalClock.StartClock(clockName.str().c_str());
    mapPassedEvaluation = true;

    for (vector<string>::iterator I = m_EvaluatorLabels.begin(); 
	 I != m_EvaluatorLabels.end(); ++I) {
      
      MapEvaluator<CfgType, WeightType>::conditional_type pEvaluator;
      pEvaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*I);
      Clock_Class EvalSubClock;
      stringstream evaluatorClockName; evaluatorClockName << "Iteration " << m_CurrentIteration << ", " << pEvaluator->GetName();
      EvalSubClock.StartClock(evaluatorClockName.str().c_str());
      cout << "\n\t";
      mapPassedEvaluation = pEvaluator->operator()(in_RegionID);
      cout << "\t";
      EvalSubClock.StopPrintClock();
      if ( mapPassedEvaluation )
	cout << "\t  (passed)\n";
      else
	cout << "\t  (failed)\n";
      if ( !mapPassedEvaluation )
	break;
    }
    EvalClock.StopPrintClock();
  } else
    mapPassedEvaluation=true;
  return mapPassedEvaluation;
}

