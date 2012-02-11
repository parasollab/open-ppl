/**
 * MARRTStrategy.cpp
 *
 * Description: Medial Axis-biased Rapidly Exploring random tree. 
 *
 * Author: Evan Greco, Kasra Manavi
 * Last Edited: 01/24/2012
 */

#include "MARRTStrategy.h"
#include "MPProblem.h"
#include "MPRegion.h"
#include "MPStrategy.h"
#include "MapEvaluator.h"
#include "Sampler.h"
#include "IOUtils.h"

MARRTStrategy::MARRTStrategy(XMLNodeReader& _node, MPProblem* _problem) :
  MPStrategyMethod(_node, _problem), m_currentIteration(0){
    ParseXML(_node);
  }

void MARRTStrategy::ParseXML(XMLNodeReader& _node) {
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr){
    if(citr->getName() == "component_connection_method"){
      string connectMethod = citr->stringXMLParameter("Method", true, "", "Component Connection Method");
      m_componentConnectors.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName() == "evaluation_method"){
      string evalMethod = citr->stringXMLParameter("Method", true, "", "Evaluation Method");
      m_evaluators.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    } 
    else
      citr->warnUnknownNode();
  }

  m_delta = _node.numberXMLParameter("delta", false, 0.05, 0.0, 1.0, "Delta Distance");
  m_minDist = _node.numberXMLParameter("minDist", false, 0.0, 0.0, 1.0, "Minimum Distance");
  m_roots = _node.numberXMLParameter("numRoots", false, 1, 0, MAX_INT, "Number of Roots");
  m_growthFocus = _node.numberXMLParameter("growthFocus", false, 0.0, 0.0, 1.0, "#GeneratedTowardsGoal/#Generated");
  m_sampler = _node.stringXMLParameter("sampler", true, "", "Sampler Method");
  m_vc = _node.stringXMLParameter("vc", true, "", "Validity Test Method");
  m_nf = _node.stringXMLParameter("nf", true, "", "Neighborhood Finder");
  m_dm = _node.stringXMLParameter("dm",true,"","Distance Metric");
  m_lp = _node.stringXMLParameter("lp", true, "", "Local Planning Method");
  m_query = _node.stringXMLParameter("query", false, "", "Query Filename");
  _node.warnUnrequestedAttributes();

  if(m_debug) PrintOptions(cout);
}

void MARRTStrategy::PrintOptions(ostream& _os) {
  typedef vector<string>::iterator SIT;
  _os << "MARRTStrategy::PrintOptions" << endl;
  _os << "\tSampler:: " << m_sampler << endl;
  _os << "\tNeighorhood Finder:: " << m_nf << endl;
  _os << "\tDistance Metric:: " << m_dm << endl;
  _os << "\tValidity Checker:: " << m_vc << endl;
  _os << "\tLocal Planner:: " << m_lp << endl;
  _os << "\tComponent Connectors:: " << endl;
  for(SIT sit = m_componentConnectors.begin(); sit!=m_componentConnectors.end(); sit++)
    _os << "\t\t" << *sit << endl;
  _os << "\tEvaluators:: " << endl;
  for(SIT sit = m_evaluators.begin(); sit!=m_evaluators.end(); sit++)
    _os << "\t\t" << *sit << endl;
  _os << "\tdelta:: " << m_delta << endl;
  _os << "\tminimum distance:: " << m_minDist << endl;
  _os << "\troots:: " << m_roots << endl;
  _os << "\tgrowth focus:: " << m_growthFocus << endl;
}

void MARRTStrategy::Initialize(int _regionID){
  if(m_debug) cout<<"\nInitializing MARRTStrategy::"<<_regionID<<endl;
  if(m_debug) cout<<"\nEnding Initializing MARRTStrategy"<<endl;
}

void MARRTStrategy::Run(int _regionID) {
  if(m_debug) cout << "\nRunning MARRTStrategy::" << _regionID << endl;
  m_debug = false;
  // Setup MP Variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  StatClass* regionStats = region->GetStatClass();
  Environment* env = region->GetRoadmap()->GetEnvironment();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  LocalPlanners<CfgType,WeightType>* lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
  ValidityChecker<CfgType>* vc = GetMPProblem()->GetValidityChecker();
  NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
  LPOutput<CfgType,WeightType> lpOutput;
  CDInfo cdInfo;
  string callee = "MARRTStrategy::MARRT";
  bool checkCollision=true, savePath=false, saveFailed=false;

  regionStats->StartClock("RRT Generation");

  // Setup RRT Variables
  CfgType tmp, dir;
  vector<CfgType> goals;
  vector<CfgType> roots;

  ifstream ifs(m_query.c_str());
  while(1){
    tmp.Read(ifs);
    if(!ifs) break;
    roots.push_back(tmp);
    tmp.Read(ifs);
    goals.push_back(tmp);
  }
  if(m_query==""){
    // Add root vertex/vertices
    for (int i=0; i<m_roots; ++i) {
      tmp.GetRandomCfg(env);
      if (tmp.InBoundingBox(env)
          && vc->IsValid(vc->GetVCMethod(m_vc), tmp, env, *regionStats, cdInfo, true, &callee)){
        roots.push_back(tmp);
      }
      else 
        --i;
    }
  }
  vector<bool> found;
  for(vector<CfgType>::iterator C = roots.begin(); C!=roots.end(); C++){
    region->GetRoadmap()->m_pRoadmap->AddVertex(*C);
    found.push_back(false);
  }

  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation){
    // Determine direction, make sure goal not found
    tmp.GetRandomCfg(env);
    double randomRatio = DRand();
    bool usingQueue = false;
    int goalNum = 0;

    if (goals.size() == 0)
      goalNum = -1;
    else if(goals.size()>1);
    goalNum = LRand()%goals.size();
    if (goals.size()>0 && randomRatio<m_growthFocus) {
      if ( found[goalNum] ) {
        for (size_t i=0; i<found.size(); i++) {
          if ( !(found[i]))
            goalNum = i;
        }
      }
      dir = goals[goalNum];
      usingQueue = true;
    } 
    else
      dir.GetRandomCfg(env);

    // Find closest Cfg in map
    vector<VID> kClosest;
    vector<VID>::iterator startKIterator;
    vector<CfgType> cfgs;
    vector<CfgType>::iterator startCIterator;

    nf->KClosest(nf->GetNFMethod(m_nf), region->GetRoadmap(), dir, 1, back_inserter(kClosest));     
    CfgType nearest = region->GetRoadmap()->m_pRoadmap->find_vertex(kClosest[0])->property();
    CfgType newCfg;
    double positionRes    = env->GetPositionRes();
    double orientationRes = env->GetOrientationRes();
    /*if(!RRTExpand(GetMPProblem(), _regionID, m_vc, m_dm, nearest, dir, newCfg, m_delta)) {
      if(m_debug) cout << "RRT could not expand!" << endl;  
      continue;
      }*/
    //Step out a distance delta towards dir cfg

    newCfg = nearest; //Start at closest node, and increment towards dir
    if(dm -> Distance(env, newCfg, dir) > m_delta){
      CfgType incr;
      incr.subtract(dir, newCfg);
      dm -> DistanceMetricMethod::ScaleCfg(env, m_delta, dir, incr);
      newCfg.Increment(incr);
    }
    else {
      newCfg = dir; //Node is already within delta, may as well use it  
    }
    CfgType tempCfg = newCfg;
    VDAddTempCfg(tempCfg, true);
    StatClass stats; 
    if (!PushToMedialAxis(GetMPProblem(), env, newCfg, stats, m_vc, m_dm, true, 20, false, 5, true, .0001, 5, m_debug, true)){
      continue;
    }
    kClosest.clear();
    //recalculate nearest Cfg; may have changed after push
    nf->KClosest(nf->GetNFMethod(m_nf), region->GetRoadmap(), newCfg, 1, back_inserter(kClosest));     
    nearest = region->GetRoadmap()->m_pRoadmap->find_vertex(kClosest[0])->property();
    // If good to go, add to roadmap
    CfgType col;
    if(dm->Distance(env, newCfg, nearest) >= m_minDist && (lp->GetMethod(m_lp)->IsConnected(env,
      *regionStats, dm, nearest, newCfg, col, &lpOutput,
              positionRes, orientationRes, checkCollision, savePath, saveFailed))) {
      cout << "Found Connection.  Nearest: " << nearest << " newCfg: " << newCfg << endl;
      VDClearAll();
      region->GetRoadmap()->m_pRoadmap->AddVertex(newCfg);
      region->GetRoadmap()->m_pRoadmap->AddEdge(nearest, newCfg, lpOutput.edge);
    }

    ConnectComponents(_regionID);
    mapPassedEvaluation = EvaluateMap(_regionID);

    // Check if goals have been found
    bool done = true;
    for(size_t i = 0; i<found.size(); i++){
      if(!found[i]){
        vector<VID> closests;
        nf->KClosest(nf->GetNFMethod(m_nf), region->GetRoadmap(), goals[i], 1, back_inserter(closests));     
        CfgType closest = region->GetRoadmap()->m_pRoadmap->find_vertex(closests[0])->property();
        double dist = dm->Distance(env, goals[i], closest);
        if(m_debug) cout << "Distance to goal::" << dist << endl;
        if(dist < m_delta && lp->GetMethod(m_lp)->IsConnected(env, *regionStats, dm, closest, goals[i], col, &lpOutput,
              positionRes, orientationRes, checkCollision, savePath, saveFailed)){
          if(m_debug) cout << "Goal found::" << goals[i] << endl;
          region->GetRoadmap()->m_pRoadmap->AddVertex(goals[i]);
          region->GetRoadmap()->m_pRoadmap->AddEdge(closest, goals[i], lpOutput.edge);
          found[i]=true;
        }
        else
          done = false;
      }
    }
    if(done){
      if(m_debug) cout << "MARRT FOUND ALL GOALS" << endl;
      mapPassedEvaluation = true;
    }
  }

  regionStats->StopClock("RRT Generation");
  if(m_debug) {
    regionStats->PrintClock("RRT Generation", cout);
    cout<<"\nEnd Running MARRTStrategy::" << _regionID << endl;  
  }
}

void MARRTStrategy::Finalize(int _regionID) {

  if(m_debug) cout<<"\nFinalizing MARRTStrategy::"<<_regionID<<endl;

  //setup region variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
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
  cout.rdbuf(osStat.rdbuf());   // redirect destination of std::cout
  cout << "NodeGen+Connection Stats" << endl;
  regionStats->PrintAllStats(osStat, region->GetRoadmap());
  regionStats->PrintClock("RRT Generation", osStat);
  //regionStats->PrintFeatures();
  osStat.close();

  if(m_debug) cout<<"\nEnd Finalizing MARRTStrategy"<<_regionID<<endl;
}

void MARRTStrategy::ConnectComponents(int _regionID) {
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  StatClass* stats = region->GetStatClass();
  MPStrategy* mps = GetMPProblem()->GetMPStrategy();
  ClockClass componentConnClock;
  string clockName = "component connection"; 
  stats->StartClock(clockName);
  stapl::sequential::vector_property_map< GRAPH,size_t > cmap;

  for(vector<string>::iterator I = m_componentConnectors.begin(); I != m_componentConnectors.end(); ++I) {
    ConnectMap<CfgType, WeightType>::ComponentConnectionPointer connector;
    connector = GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetComponentMethod(*I);

    ClockClass componentConnSubClock;
    string connectorClockName = connector->GetName();
    stats->StartClock(connectorClockName);

    if(m_debug) cout << "\n\t";
    mps->GetConnectMap()->ConnectComponents(connector, 
        region->GetRoadmap(), 
        *(region->GetStatClass()), 
        mps->addPartialEdge, 
        mps->addAllEdges);

    cmap.reset();
    if(m_debug)
      cout << region->GetRoadmap()->m_pRoadmap->get_num_edges() << " edges, " 
        << get_cc_count(*(region->GetRoadmap()->m_pRoadmap), cmap) << " connected components"
        << "\n\t";

    stats->StopClock(connectorClockName);
    if(m_debug) stats->PrintClock(connectorClockName, cout);
  }
  stats->StopClock(clockName);
  if(m_debug) stats->PrintClock(clockName, cout);
}

bool MARRTStrategy::EvaluateMap(int _regionID) {
  if (m_evaluators.empty()) {
    return true;
  }
  else{
    MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
    StatClass* stats = region->GetStatClass();
    bool mapPassedEvaluation = false;
    ClockClass evalClock;
    string clockName = "Map Evaluation"; 
    stats->StartClock(clockName);
    mapPassedEvaluation = true;

    for (vector<string>::iterator I = m_evaluators.begin(); I != m_evaluators.end(); ++I) {
      MapEvaluator<CfgType, WeightType>::MapEvaluationMethodPtr evaluator;
      evaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*I);
      ClockClass evalSubClock;
      string evaluatorClockName = evaluator->GetName();
      stats->StartClock(evaluatorClockName);
      if(m_debug) cout << "\n\t";
      mapPassedEvaluation = evaluator->operator()(_regionID);
      if(m_debug) cout << "\t";
      stats->StopClock(evaluatorClockName);
      if(m_debug) stats->PrintClock(evaluatorClockName, cout);
      if(mapPassedEvaluation){
        if(m_debug) cout << "\t  (passed)\n";
      }
      else{
        if(m_debug) cout << "\t  (failed)\n";
      }
      if(!mapPassedEvaluation)
        break;
    }
    stats->StopClock(clockName);
    if(m_debug) stats->PrintClock(clockName, cout);
    return mapPassedEvaluation;
  } 
}

