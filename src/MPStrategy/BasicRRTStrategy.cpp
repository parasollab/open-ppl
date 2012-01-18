/**
 * BasicRRTStrategy.cpp
 *
 * Description: Main RRT Strategy, contains RRT method used in RRTconnect
 *
 * Author: Kasra Manavi
 * Last Edited: 04/08/2011
 */

#include "BasicRRTStrategy.h"
#include "MPProblem.h"
#include "MPRegion.h"
#include "MPStrategy.h"
#include "MapEvaluator.h"
#include "Sampler.h"

BasicRRTStrategy::BasicRRTStrategy(XMLNodeReader& _node, MPProblem* _problem) :
  MPStrategyMethod(_node, _problem), m_currentIteration(0){
    ParseXML(_node);
  }

void BasicRRTStrategy::ParseXML(XMLNodeReader& _node) {
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

void BasicRRTStrategy::PrintOptions(ostream& _os) {
  typedef vector<string>::iterator SIT;
  _os << "BasicRRTStrategy::PrintOptions" << endl;
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

void BasicRRTStrategy::Initialize(int _regionID){
  if(m_debug) cout<<"\nInitializing BasicRRTStrategy::"<<_regionID<<endl;
  if(m_debug) cout<<"\nEnding Initializing BasicRRTStrategy"<<endl;
}

void BasicRRTStrategy::Run(int _regionID) {
  if(m_debug) cout << "\nRunning BasicRRTStrategy::" << _regionID << endl;

  // Setup MP Variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  StatClass* regionStats = region->GetStatClass();
  Environment* env = region->GetRoadmap()->GetEnvironment();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetDMMethod(m_dm);
  LocalPlanners<CfgType,WeightType>* lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
  ValidityChecker<CfgType>* vc = GetMPProblem()->GetValidityChecker();
  NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
  LPOutput<CfgType,WeightType> lpOutput;
  CDInfo cdInfo;
  string callee = "BasicRRTStrategy::RRT";
  bool checkCollision=false, savePath=false, saveFailed=false;

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

    if(!RRTExpand(GetMPProblem(), _regionID, m_vc, m_dm, nearest, dir, newCfg, m_delta)) {
      if(m_debug) cout << "RRT could not expand!" << endl;  
      continue;
    }
    // If good to go, add to roadmap
    double positionRes    = env->GetPositionRes();
    double orientationRes = env->GetOrientationRes();
    CfgType col;
    if(dm->Distance(env, newCfg, nearest) >= m_minDist) {
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
        if(dist < m_delta && lp->GetLocalPlannerMethod(m_lp)->IsConnected(env, *regionStats, dm, closest, goals[i], col, &lpOutput,
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
      if(m_debug) cout << "RRT FOUND ALL GOALS" << endl;
      mapPassedEvaluation = true;
    }
 }

  regionStats->StopClock("RRT Generation");
  if(m_debug) {
    regionStats->PrintClock("RRT Generation", cout);
    cout<<"\nEnd Running BasicRRTStrategy::" << _regionID << endl;  
  }
}

void BasicRRTStrategy::Finalize(int _regionID) {

  if(m_debug) cout<<"\nFinalizing BasicRRTStrategy::"<<_regionID<<endl;

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
  osStat << "NodeGen+Connection Stats" << endl;
  regionStats->PrintAllStats(osStat, region->GetRoadmap());
  regionStats->PrintClock("RRT Generation", osStat);
  osStat.close();

  if(m_debug) cout<<"\nEnd Finalizing BasicRRTStrategy"<<_regionID<<endl;
}

void BasicRRTStrategy::ConnectComponents(int _regionID) {
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  StatClass* stats = region->GetStatClass();
  MPStrategy* mps = GetMPProblem()->GetMPStrategy();
  stringstream clockName; clockName << "Iteration " << m_currentIteration << ", Component Connection";
  stats->StartClock(clockName.str());
  stapl::vector_property_map< GRAPH,size_t > cmap;

  for(vector<string>::iterator I = m_componentConnectors.begin(); I != m_componentConnectors.end(); ++I) {
    ConnectMap<CfgType, WeightType>::ComponentConnectionPointer connector;
    connector = GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetComponentMethod(*I);

    stringstream connectorClockName; connectorClockName << "Iteration " << m_currentIteration << ", " << connector->GetName();
    stats->StartClock(connectorClockName.str());

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

    stats->StopClock(connectorClockName.str());
    if(m_debug) stats->PrintClock(connectorClockName.str(), cout);
  }
  stats->StopClock(clockName.str());
  if(m_debug) stats->PrintClock(clockName.str(), cout);
}

bool BasicRRTStrategy::EvaluateMap(int _regionID) {
  if (m_evaluators.empty()) {
    return true;
  }
  else{
    MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
    StatClass* stats = region->GetStatClass();
    
    bool mapPassedEvaluation = false;
    stringstream clockName; clockName << "Iteration " << m_currentIteration << ", Map Evaluation"; 
    stats->StartClock(clockName.str());
    mapPassedEvaluation = true;

    for (vector<string>::iterator I = m_evaluators.begin(); I != m_evaluators.end(); ++I) {
      MapEvaluator<CfgType, WeightType>::conditional_type evaluator;
      evaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*I);
      stringstream evaluatorClockName; evaluatorClockName << "Iteration " << m_currentIteration << ", " << evaluator->GetName();
      stats->StartClock(evaluatorClockName.str());
      if(m_debug) cout << "\n\t";
      mapPassedEvaluation = evaluator->operator()(_regionID);
      if(m_debug) cout << "\t";
      stats->StopClock(evaluatorClockName.str());
      if(m_debug) stats->PrintClock(evaluatorClockName.str(), cout);
      if(mapPassedEvaluation){
        if(m_debug) cout << "\t  (passed)\n";
      }
      else{
        if(m_debug) cout << "\t  (failed)\n";
      }
      if(!mapPassedEvaluation)
        break;
    }
    stats->StopClock(clockName.str());
    if(m_debug) stats->PrintClock(clockName.str(), cout);
    return mapPassedEvaluation;
  } 
}

