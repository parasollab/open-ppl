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
  m_exact = _node.boolXMLParameter("exact", false, "", "Exact Medial Axis Calculation");
  m_rayCount = _node.numberXMLParameter("rays", false, 20, 0, 50, "Number of Clearance Rays");
  m_penetration = _node.numberXMLParameter("penetration", false, 5, 0, 50, "Pentration");
  m_useBbx = _node.boolXMLParameter("useBBX", true, "", "Use Bounding Box");
  m_hLen = _node.numberXMLParameter("hLen", false, 5, 0, 20, "History Length");
  m_positional = _node.boolXMLParameter("positional", true, "", "Use Position in MA Calculations");
  m_debug = _node.boolXMLParameter("debug", false, "", "Debug Mode");
  m_findQuery = _node.boolXMLParameter("findQuery", true, "", "Find Query or Explore Space");
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

  ifstream ifs(m_query.c_str());
  while(1){
    tmp.Read(ifs);
    if(!ifs) break;
    m_root.push_back(tmp);
    tmp.Read(ifs);
    m_goals.push_back(tmp);
    if(m_debug) cout << "New Goal: " << tmp << endl;
  }
  if(m_query==""){
    // Add root vertex/vertices
    for (int i=0; i<m_roots; ++i) {
      tmp.GetRandomCfg(env);
      if (tmp.InBoundingBox(env)
          && vc->IsValid(vc->GetVCMethod(m_vc), tmp, env, *regionStats, cdInfo, true, &callee)){
        m_root.push_back(tmp);
      }
      else 
        --i;
    }
  }
  vector<bool> found;
  for(vector<CfgType>::iterator C = m_root.begin(); C!=m_root.end(); C++){
    region->GetRoadmap()->m_pRoadmap->AddVertex(*C);
    found.push_back(false);
  }

  bool mapPassedEvaluation = false;
  CfgType currentClosest = m_root[0];
  int numSampled=0;
  while(!mapPassedEvaluation){
    // Determine direction, make sure goal not found
    tmp.GetRandomCfg(env);
    double randomRatio = DRand();
    bool usingQueue = false;
    int goalNum = 0;

    if (m_goals.size() == 0)
      goalNum = -1;
    else if(m_goals.size()>1);
    goalNum = LRand()%m_goals.size();
    if (m_goals.size()>0 && randomRatio<m_growthFocus) {
      if ( found[goalNum] ) {
        for (size_t i=0; i<found.size(); i++) {
          if ( !(found[i]))
            goalNum = i;
        }
      }
      dir = m_goals[goalNum];
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

    newCfg = nearest;
    CDInfo dummyCD;
    if(!RRTExpand(GetMPProblem(), _regionID, m_vc, m_dm, newCfg, dir, newCfg, m_delta, dummyCD)) {
      if(m_debug) cout << "RRT could not expand!" << endl;  
      continue;
    }
    //Step out a distance delta towards dir cfg

    /*newCfg = nearest; //Start at closest node, and increment towards dir
      if(dm -> Distance(env, newCfg, dir) > m_delta){
      CfgType incr;
      incr.subtract(dir, newCfg);
      dm -> DistanceMetricMethod::ScaleCfg(env, m_delta, dir, incr);
      newCfg.Increment(incr);
      }
      else {
      newCfg = dir; //Node is already within delta, may as well use it  
      }*/
    CfgType tempCfg = newCfg;
    VDAddTempCfg(tempCfg, true);
    StatClass stats; 
    if (!PushToMedialAxis(GetMPProblem(), env, newCfg, stats, m_vc, m_dm, m_exact, m_rayCount, m_exact, m_penetration, m_useBbx, .0001, m_hLen, m_debug, m_positional)){
      continue;
    }
    VDAddTempCfg(newCfg, true);
    kClosest.clear();
    //recalculate nearest Cfg; may have changed after push
    nf->KClosest(nf->GetNFMethod(m_nf), region->GetRoadmap(), newCfg, 1, back_inserter(kClosest));     
    nearest = region->GetRoadmap()->m_pRoadmap->find_vertex(kClosest[0])->property();
    // If good to go, add to roadmap
    CfgType col;
    bool newConnectionFound = false;
    if((lp->GetMethod(m_lp)->IsConnected(env,
            *regionStats, dm, newCfg, nearest, col, &lpOutput,
            positionRes, orientationRes, checkCollision, savePath, saveFailed))) {
      newConnectionFound=true;
      cout << "Found Connection.  Nearest: " << nearest << " newCfg: " << newCfg << endl;
      numSampled++;
      cout << "Number of nodes sampled thus far: " << numSampled << endl;
      VDClearAll();
      region->GetRoadmap()->m_pRoadmap->AddVertex(newCfg);
      region->GetRoadmap()->m_pRoadmap->AddEdge(nearest, newCfg, lpOutput.edge);
    }

    ConnectComponents(_regionID);
    mapPassedEvaluation = EvaluateMap(_regionID);

    // Check if goals have been found
    bool done = true;
    if(m_findQuery){
      if(newConnectionFound) {
        for(size_t i = 0; i<found.size(); i++){
          if(!found[i]){
            vector<VID> closests;
            nf->KClosest(nf->GetNFMethod(m_nf), region->GetRoadmap(), m_goals[i], 1, back_inserter(closests));     
            CfgType closest = region->GetRoadmap()->m_pRoadmap->find_vertex(closests[0])->property();
            if(closest != currentClosest){
              currentClosest = closest; //Currently closest to goal.  No need to check for connectivity if we're no closer to the goal
              double dist = dm->Distance(env, m_goals[i], closest);
              cout << "Distance to goal::" << dist << endl;
              if(lp->GetMethod(m_lp)->IsConnected(env, *regionStats, dm, closest, m_goals[i], col, &lpOutput,
                    positionRes, orientationRes, false, savePath, saveFailed)){
                if(m_debug) cout << "Goal found::" << m_goals[i] << endl;
                region->GetRoadmap()->m_pRoadmap->AddVertex(m_goals[i]);
                region->GetRoadmap()->m_pRoadmap->AddEdge(closest, m_goals[i], lpOutput.edge);
                found[i]=true;
              }
              else
                done = false;
            }
            else{
              done=false;
            }
          }
        }
      }
      else{
        done = false; //No new connection, can't find goal, not done.
      }
      if(done){
        if(m_debug) cout << "MARRT FOUND ALL GOALS" << endl;
        mapPassedEvaluation = true;
        m_queryFound = true;
      }
    }
  }

  regionStats->StopClock("RRT Generation");
  if(m_debug) {
    regionStats->PrintClock("RRT Generation", cout);
    if(m_debug) cout<<"\nEnd Running MARRTStrategy::" << _regionID << endl;  
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
  if(m_debug) cout << "NodeGen+Connection Stats" << endl;
  regionStats->PrintAllStats(osStat, region->GetRoadmap());
  regionStats->PrintClock("RRT Generation", osStat);
  RoadmapClearanceStats clearanceStats = RoadmapClearance(GetMPProblem(), false, region->GetRoadmap()->GetEnvironment(), *region->GetRoadmap(), m_vc, m_dm);
  osStat << endl <<  "Min Roadmap Clearance: " << clearanceStats.m_minClearance << endl <<  " Avg Roadmap Clearance: " << clearanceStats.m_avgClearance << endl << " Roadmap Variance: " << clearanceStats.m_clearanceVariance << endl;
  //regionStats->PrintFeatures();
  if(m_queryFound){
  RoadmapClearanceStats pathStats = PathClearance(_regionID);
  osStat << endl << "Path Length: " << pathStats.m_pathLength << endl << "Min Path Clearance: " << pathStats.m_minClearance << endl << " Avg Path Clearance: " << pathStats.m_avgClearance << endl << " Path Variance: " << pathStats.m_clearanceVariance << endl;
  }

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
    Connector<CfgType, WeightType>::ConnectionPointer connector;
    connector = GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*I);

    ClockClass componentConnSubClock;
    string connectorClockName = connector->GetName();
    stats->StartClock(connectorClockName);

    if(m_debug) cout << "\n\t";
    mps->GetConnector()->Connect(connector, 
        region->GetRoadmap(), 
        *(region->GetStatClass()));
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

RoadmapClearanceStats 
MARRTStrategy::PathClearance(int _regionID){
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  RoadmapGraph<CfgType, WeightType>* graph = region->GetRoadmap()->m_pRoadmap;
  int svid = graph->GetVID(m_root[0]);
  int gvid = graph->GetVID(m_goals[0]);
  vector<VID> path;
  find_path_dijkstra(*(graph), svid, gvid, path, WeightType::MaxWeight());
  RoadmapClearanceStats stats;
  typedef RoadmapGraph<CfgType, WeightType>::EI EI;
  typedef RoadmapGraph<CfgType, WeightType>::VI VI;
  typedef RoadmapGraph<CfgType, WeightType>::EID EID;
  double runningTotal = 0;
  double minClearance = 1e6;
  double pathLength = 0;
  vector<double> clearanceVec;
  for(size_t i = 0; i < path.size() - 1; i++){
    EI ei;
    VI vi;
    EID ed(path[i], path[i+1]);
    graph->find_edge(ed, vi, ei);
    WeightType weight = (*ei).property();
    pathLength += weight.Weight();
    double currentClearance = MinEdgeClearance(GetMPProblem(), false, region->GetRoadmap()->GetEnvironment(), (*graph->find_vertex((*ei).source())).property(), (*graph->find_vertex((*ei).target())).property(), weight, m_vc, m_dm); 
    clearanceVec.push_back(currentClearance);
    runningTotal += currentClearance;
    if(currentClearance < minClearance){
      minClearance = currentClearance;    
    }
  }
  stats.m_minClearance = minClearance;
  double average = runningTotal / (path.size()/2);
  stats.m_avgClearance = average;
  double varSum = 0;
  for(vector<double>::iterator it = clearanceVec.begin(); it != clearanceVec.end(); it++){
    varSum+=pow(((*it) - average), 2);  
  }
  stats.m_clearanceVariance = varSum / clearanceVec.size();
  stats.m_pathLength = pathLength;
  return stats;

//RoadmapClearanceStats clearanceStats = RoadmapClearance(GetMPProblem(), false, region->GetRoadmap()->GetEnvironment(), *region->GetRoadmap(), m_vc, m_dm);
  
  
  
}



