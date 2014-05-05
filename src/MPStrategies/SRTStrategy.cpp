/**
 * SRTStrategy.cpp
 *
 * Description: Main RRT Strategy, contains RRT method used in RRTconnect
 *
 * Author: Kasra Manavi
 * Last Edited: 04/08/2011
 */

#include "MPStrategy/SRTStrategy.h"
#include "MPProblem/MPRegion.h"
#include "MPStrategy/MPStrategy.h"

SRTStrategy::SRTStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool isInherited) :
  MPStrategyMethod(in_Node, in_pProblem), m_CurrentIteration(0){
  if(!isInherited)
    ParseXML(in_Node);
}

SRTStrategy::~SRTStrategy(){ }

#include "boost/lambda/lambda.hpp"

void SRTStrategy::ParseXML(XMLNodeReader& in_Node) {
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
      roots       = citr->numberXMLParameter(string("numRoots"), false, 1, 1, 1000, string("Number of Roots"));
      growthFocus = citr->numberXMLParameter(string("growthFocus"), false, 0.00, 0.0, 1.0, string("#GeneratedTowardsGoal/#Generated"));
      attempts    = citr->boolXMLParameter(string("attempts"), true, false, string("Count attempts, not generated"));
      citr->warnUnrequestedAttributes();
    } else
      citr->warnUnknownNode();
  }

  using boost::lambda::_1;
  cout << "\nSRTStrategy::ParseXML:\n";
  cout << "\tnode_generation_methods: ";
  for(vector<pair<string, int> >::iterator I = m_NodeGenerationLabels.begin(); I!=m_NodeGenerationLabels.end(); I++)
    cout<<I->first<<"\tNumber:"<<I->second<<" ";
  cout << endl;
  cout << "\tdelta:      " << delta << endl;
  cout << "\tminDist:    " << minDist << endl;
  cout << "\obsDist:     " << obsDist << endl;
  cout << "\numRoots:    " << roots << endl;
  cout << "\growthFocus: " << growthFocus << endl;
  cout << "\tnode_connection_methods: "; for_each(m_NodeConnectionLabels.begin(), m_NodeConnectionLabels.end(), cout << _1 << " "); cout << endl;
  cout << "\tcomponent_connection_methods: "; for_each(m_ComponentConnectionLabels.begin(), m_ComponentConnectionLabels.end(), cout << _1 << " "); cout << endl;
  cout << "\tevaluator_methods: "; for_each(m_EvaluatorLabels.begin(), m_EvaluatorLabels.end(), cout << _1 << " "); cout << endl;
  cout << "\tlp_method: " << m_LPMethod << endl;
  cout << "\tdm_method: " << dm_label << endl;
  cout << "\tnf_method: " << nf_label << endl;
  cout << "\tstrVcmethod = " << strVcmethod << endl;
}

void SRTStrategy::PrintOptions(ostream& out_os) const {
  out_os<<"SRTStrategy::PrintOptions()\n";
  out_os<<"\nNodeGenerators\n";
  typedef vector<pair<string,int> >::iterator PIT;
  typedef vector<string>::iterator SIT;
  for(PIT pit=m_NodeGenerationLabels.begin(); pit!=m_NodeGenerationLabels.end(); pit++){
    out_os<<"\t"<<pit->first<<"\tNumber of Samples Per Iteration:"<<pit->second<<"\tOptions:\n";
    GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(pit->first)->PrintOptions(out_os);
  }
  out_os<<"\nNodeConnectors\n";
  for(SIT sit=m_NodeConnectionLabels.begin(); sit!=m_NodeConnectionLabels.end(); sit++){
    out_os<<"\t"<<*sit<<"\tOptions:\n";
    GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*sit)->PrintOptions(out_os);
  }
  out_os<<"\nLocalPlanner\n";
  out_os<<"\t"<<m_LPMethod<<"\tOptions:\n";
  GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_LPMethod)->PrintOptions(out_os);
  out_os<<"\nComponentConnectors\n";
  for(SIT sit=m_ComponentConnectionLabels.begin(); sit!=m_ComponentConnectionLabels.end(); sit++){
    out_os<<"\t"<<*sit<<"\tOptions:\n";
    GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*sit)->PrintOptions(out_os);
  }
  out_os<<"\nMapEvaluators\n";
  for(SIT sit=m_EvaluatorLabels.begin(); sit!=m_EvaluatorLabels.end(); sit++){
    out_os<<"\t"<<*sit<<"\tOptions:\n";
    //GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*sit)->PrintOptions(out_os);
  }
}

void SRTStrategy::Initialize(int in_RegionID){
   cout<<"\nInitializing SRTStrategy::"<<in_RegionID<<endl;
   //OBPRM_srand(getSeed());
   cout<<"\nEnding Initializing SRTStrategy"<<endl;
}

void SRTStrategy::Run(int in_RegionID) {
  //cout << "\nRunning SRTStrategy::" << in_RegionID << endl;
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
  vector<CfgType> goals;
  bool mapPassedEvaluation = false;
  vector<pair<CfgType,vector<VID> > >* pairs;
  RRTStrategy(in_RegionID, goals, pairs);
  //ConnectComponents(region);
  //cout <<"\nEnd Running SRTStrategy::" << in_RegionID << endl;
}

void SRTStrategy::Run(int in_RegionID, //RoadmapGraph<CfgType,WeightType>* candGraph,
		      vector<pair<CfgType,vector<VID> > >* trees) {
  //cout << "\nRunning SRTStrategy::" << in_RegionID << endl;
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
  vector<CfgType> goals;
  bool mapPassedEvaluation = false;
  RRTStrategy(in_RegionID, goals, trees);
  //ConnectComponents(region);
  //cout << "\nEnd Running SRTStrategy::" << in_RegionID << endl;
}

// MAIN RRT METHOD
void SRTStrategy::RRTStrategy(int in_RegionID, vector<CfgType> RRTQueue,
			      //RoadmapGraph<CfgType,WeightType>* candGraph,
			      vector<pair<CfgType,vector<VID> > >* trees) {

  //cout << "In SRTStrategy::RRTStrategy num_nodes = " << num_nodes << endl;
  // Setup MP Variables
  MPRegion<CfgType,WeightType>*      region = GetMPProblem()->GetMPRegion(in_RegionID);
  //StatClass*                        regionStats = region->GetStatClass();
  Environment*                       env = region->GetRoadmap()->GetEnvironment();
  shared_ptr <DistanceMetricMethod>  _dm = GetMPProblem()->GetDistanceMetric()->GetMethod(dm_label);
  LocalPlanners<CfgType,WeightType>* lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
  ValidityChecker<CfgType>*          vc = GetMPProblem()->GetValidityChecker();
  LPOutput<CfgType,WeightType>       lpOutput;
  CDInfo                             cdInfo;
  cdInfo.ret_all_info = false;
  string callee("SRTStrategy::RRT");
//TODO: replace with RRTExpand from BasicRRTStrategy
//RRTconnect<CfgType,WeightType>* RRTCon = new RRTconnect<CfgType,WeightType>(this->Get    MPProblem());

  typedef vector<pair<CfgType,vector<VID> > > cPairs;
  cPairs candPairs;

  // Setup RRT Variables
  CfgType tmp, dir;
  bool connecting=true, checkCollision=false, savePath=false, saveFailed=false, mapPassed=false;
  vector<bool> found;

  for (int j=0; j<RRTQueue.size(); ++j)
    found.push_back(false);

  MapGenClock.StartClock();

  // For the number of iterations
  for (int h=0; h<m_CurrentIteration; ++h) {
    // Main loop for generation
    for (int i=0; i<roots; ++i) {
      do {
	tmp.GetRandomCfg(env,m_boundary);
      } while( !tmp.InBoundingBox(env, m_boundary) ||
               !this->GetMPProblem()->GetValidityChecker()->GetMethod(strVcmethod)->IsValid(tmp, cdInfo, true, &callee));

      CfgType root = CfgType(tmp);//, cent = CfgType(tmp);
      CfgType centroidCFG = CfgType(tmp);
      vector<VID> tree = vector<VID>();
      CfgType* cent = &centroidCFG;
      //centroid = CfgType(tmp);

      VID tempVID, centroidVID;
      tempVID     = region->GetRoadmap()->m_pRoadmap->AddVertex(root);
      //cout << "Added Root VID = " << tempVID << endl;
      centroidVID = tempVID;

      //cout << "SRTStrategy::RRTStrategy: (tempVID) = centroidVID =  " << centroidVID << endl;
      //cout << "SRTStrategy::RRTStrategy: centroidCFG =  " << centroidCFG << endl;
/*
      RRTExpand(this->GetMPProblem, region->GetRoadmap(),m_boundary,
			*regionStats,lp,tempVID,
                        //num_nodes/
                        roots-1,
			//num_nodes/
                        roots,delta,obsDist,minDist,
			this->GetMPProblem()->GetDistanceMetric()->GetMethod(dm_label),
			this->GetMPProblem()->GetValidityChecker(), strVcmethod,
			this->GetMPProblem()->GetCollisionDetection(),
			this->GetMPProblem()->GetNeighborhoodFinder(), nf_label,
			tree, centroidCFG,attempts);*/
      //cout << "SRTStrategy::RRTStrategy: centroidCFG: " << centroidCFG << endl;
      //centroidVID = candGraph->AddVertex(centroidCFG);
      //cout << "SRTStrategy::RRTStrategy - Centroid to Add: " << &cent << endl;
      //centroidVID = candGraph->AddVertex(&cent);
      //cout << "SRTStrategy::RRTStrategy: centroidVID: " << centroidVID << endl;
      // Evalutate
      //mapPassed = EvaluateMap(in_RegionID);
      candPairs.push_back(pair<CfgType,vector<VID> >(centroidCFG,tree));
      //if (mapPassed)
      //cout << "Map Evaluator Passed... Iteration = " << h << endl;
    }
  }
  typedef vector<pair<CfgType,vector<VID> > >::iterator cand_iter;
  for (cand_iter c = candPairs.begin(); c != candPairs.end(); ++c) {
    //cout << "Paired VIDs: " << c->first << "/" << c->second << endl;
    trees->push_back(*c);
  }
}

void SRTStrategy::Finalize(int in_RegionID) {

  cout<<"\nFinalizing SRTStrategy::"<<in_RegionID<<endl;

  //setup region variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
  StatClass* regionStats = region->GetStatClass();
  string str;

  //output final map
  str = GetBaseFilename() + ".map";
  ofstream osMap(str.c_str());
  region->WriteRoadmapForVizmo(osMap);
  osMap.close();

  //output stats
  str = GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  streambuf* sbuf = cout.rdbuf(); // to be restored later
  cout.rdbuf(osStat.rdbuf());   // redirect destination of std::cout
  cout << "NodeGen+Connection Stats" << endl;
  regionStats->PrintAllStats(osStat, region->GetRoadmap());
  MapGenClock.PrintClock(osStat);
  //regionStats->PrintFeatures();
  cout.rdbuf(sbuf);  // restore original stream buffer
  osStat.close();

  cout<<"\nEnd Finalizing SRTStrategy"<<endl;
}

void SRTStrategy::ConnectComponents(MPRegion<CfgType, WeightType>* region) {

  ClockClass ComponentConnClock;
  stringstream clockName; clockName << "Iteration " << m_CurrentIteration << ", Component Connection";
  ComponentConnClock.StartClock();//clockName.str().c_str()
  stapl::sequential::vector_property_map< GRAPH,size_t > cmap;

  for (vector<string>::iterator I = m_ComponentConnectionLabels.begin();
       I != m_ComponentConnectionLabels.end(); ++I) {
    Connector<CfgType, WeightType>::ConnectionPointer pConnection;
    pConnection = GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*I);

    ClockClass ComponentConnSubClock;
    stringstream connectorClockName; connectorClockName << "Iteration " << m_CurrentIteration << ", " << pConnection->GetName();
    ComponentConnSubClock.StartClock(); //connectorClockName.str().c_str()

    cout << "\n\t";
    vector<CfgType> collision;

    cmap.reset();
    cout << region->GetRoadmap()->m_pRoadmap->get_num_edges() << " edges, "
	 << get_cc_count(*(region->GetRoadmap()->m_pRoadmap), cmap) << " connected components"
	 << endl;

    cout << "\t";
    ComponentConnSubClock.StopPrintClock(connectorClockName);
  }
  ComponentConnClock.StopPrintClock(clockName);
}


bool SRTStrategy::EvaluateMap(int in_RegionID) {

  bool mapPassedEvaluation = false;
  if ( !m_EvaluatorLabels.empty() ) {

    ClockClass EvalClock;
    stringstream clockName; clockName << "Iteration " << m_CurrentIteration << ", Map Evaluation";
    EvalClock.StartClock();//clockName.str().c_str()
    mapPassedEvaluation = true;

    for (vector<string>::iterator I = m_EvaluatorLabels.begin();
	 I != m_EvaluatorLabels.end(); ++I) {

      MapEvaluator<CfgType, WeightType>::MapEvaluationMethodPtr pEvaluator;
      //pEvaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*I);
      ClockClass EvalSubClock;
      stringstream evaluatorClockName; evaluatorClockName << "Iteration " << m_CurrentIteration << ", " << pEvaluator->GetName();
      EvalSubClock.StartClock(); //evaluatorClockName.str().c_str()
      cout << "\n\t";
      //mapPassedEvaluation = pEvaluator->operator()(in_RegionID);
      cout << "\t";
      EvalSubClock.StopPrintClock(clockName);
      if ( mapPassedEvaluation )
	cout << "\t  (passed)\n";
      else
	cout << "\t  (failed)\n";
      if ( !mapPassedEvaluation )
	break;
    }
    EvalClock.StopPrintClock(clockName);
  } else
    mapPassedEvaluation=true;
  return mapPassedEvaluation;
}

