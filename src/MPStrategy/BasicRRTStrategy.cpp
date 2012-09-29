//////////////////////////
//BasicRRTStrategy cpp file
//
//Description: Main RRt Strategy, contain RRT method used in RRTconnect
/////////////////////////

#include "BasicRRTStrategy.h"
#include "MPProblem.h"
#include "MPStrategy.h"
#include "MapEvaluator.h"
#include "Sampler.h"

///////////////////////
//Constructors
//////////////////////
BasicRRTStrategy::BasicRRTStrategy(XMLNodeReader& _node, MPProblem* _problem, bool _warnXML) :
  MPStrategyMethod(_node, _problem), m_currentIteration(0){
    ParseXML(_node);
    if (_warnXML) _node.warnUnrequestedAttributes();
    if(m_debug && _warnXML) PrintOptions(cout);
  }

void BasicRRTStrategy::ParseXML(XMLNodeReader& _node) {
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr){
    if(citr->getName() == "evaluation_method"){
      string evalMethod = citr->stringXMLParameter("Method", true, "", "Evaluation Method");
      m_evaluators.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    } 
    else
      citr->warnUnknownNode();
  }

  m_delta = _node.numberXMLParameter("delta", false, 0.05, 0.0, 1.0, "Delta Distance");
  m_minDist = _node.numberXMLParameter("minDist", false, 0.0, 0.0, 1.0, "Minimum Distance");
  m_numRoots = _node.numberXMLParameter("numRoots", false, 1, 0, MAX_INT, "Number of Roots");
  m_growthFocus = _node.numberXMLParameter("growthFocus", false, 0.0, 0.0, 1.0, "#GeneratedTowardsGoal/#Generated");
  m_sampler = _node.stringXMLParameter("sampler", true, "", "Sampler Method");
  m_vc = _node.stringXMLParameter("vc", true, "", "Validity Test Method");
  m_nf = _node.stringXMLParameter("nf", true, "", "Neighborhood Finder");
  m_dm = _node.stringXMLParameter("dm",true,"","Distance Metric");
  m_lp = _node.stringXMLParameter("lp", true, "", "Local Planning Method");
  m_query = _node.stringXMLParameter("query", false, "", "Query Filename");
}

void BasicRRTStrategy::PrintOptions(ostream& _os) {
  typedef vector<string>::iterator SIT;
  _os << "BasicRRTStrategy::PrintOptions" << endl;
  _os << "\tSampler:: " << m_sampler << endl;
  _os << "\tNeighorhood Finder:: " << m_nf << endl;
  _os << "\tDistance Metric:: " << m_dm << endl;
  _os << "\tValidity Checker:: " << m_vc << endl;
  _os << "\tLocal Planner:: " << m_lp << endl;
  _os << "\tEvaluators:: " << endl;
  for(SIT sit = m_evaluators.begin(); sit!=m_evaluators.end(); sit++)
    _os << "\t\t" << *sit << endl;
  _os << "\tdelta:: " << m_delta << endl;
  _os << "\tminimum distance:: " << m_minDist << endl;
  _os << "\tnumber of roots:: " << m_numRoots << endl;
  _os << "\tgrowth focus:: " << m_growthFocus << endl;
}

//////////////////////
//Initialization Phase
/////////////////////
void 
BasicRRTStrategy::Initialize(){
  if(m_debug) cout<<"\nInitializing BasicRRTStrategy::"<<endl;

  // Setup MP variables
  StatClass* stats = GetMPProblem()->GetStatClass();
  Environment* env = GetMPProblem()->GetRoadmap()->GetEnvironment();
  ValidityChecker* vc = GetMPProblem()->GetValidityChecker();
  CDInfo cdInfo;
  string callee = "BasicRRTStrategy::RRT";
  // Setup RRT Variables
  CfgType tmp;
  if(m_query != ""){
    ifstream ifs(m_query.c_str());
    while(1){
      tmp.Read(ifs);
      if(!ifs) break;
      m_roots.push_back(tmp);
      tmp.Read(ifs);
      m_goals.push_back(tmp);
      m_goalsNotFound.push_back(m_goals.size()-1);
    }
  }
  else{
    // Add root vertex/vertices
    for (int i=0; i<m_numRoots; ++i) {
      tmp.GetRandomCfg(env);
      if (tmp.InBoundary(env)
          && vc->GetMethod(m_vc)->IsValid(tmp, env, *stats, cdInfo, &callee)){
        m_roots.push_back(tmp);
        m_goals.push_back(tmp);
        m_goalsNotFound.push_back(m_goals.size()-1);
      }
      else 
        --i;
    }
  }
  for(vector<CfgType>::iterator C = m_roots.begin(); C!=m_roots.end(); C++){
    GetMPProblem()->GetRoadmap()->m_pRoadmap->AddVertex(*C);
  }

  if(m_debug) cout<<"\nEnding Initializing BasicRRTStrategy"<<endl;
}

////////////////
//Run/Start Phase
////////////////
void BasicRRTStrategy::Run() {
  if(m_debug) cout << "\nRunning BasicRRTStrategy::" << endl;

  // Setup MP Variables
  StatClass* stats = GetMPProblem()->GetStatClass();

  stats->StartClock("RRT Generation");

  CfgType dir;
  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation){
    //find my growth direction. Default is too randomly select node or bias towards a goal
    double randomRatio = DRand();
    if(randomRatio<m_growthFocus){
      dir = GoalBiasedDirection();
    }
    else{
      dir = this->SelectDirection();
    }

    //grow towards the direction
    VID recent = this->ExpandTree(dir);
    if(recent != INVALID_VID){
      //conntect various trees together
      ConnectTrees(recent);
      //see if tree is connected to goals
      EvaluateGoals();
    }
    
    //evaluate the roadmap
    mapPassedEvaluation = EvaluateMap(m_evaluators);

    if(m_goalsNotFound.size()==0){
      if(m_debug) cout << "RRT FOUND ALL GOALS" << endl;
      mapPassedEvaluation = true;
    }
  }

  stats->StopClock("RRT Generation");
  if(m_debug) {
    stats->PrintClock("RRT Generation", cout);
    cout<<"\nEnd Running BasicRRTStrategy::" << endl;  
  }
}

/////////////////////
//Finalization phase
////////////////////
void BasicRRTStrategy::Finalize() {
 
  if(m_debug) cout<<"\nFinalizing BasicRRTStrategy::"<<endl;

  //setup variables
  StatClass* stats = GetMPProblem()->GetStatClass();
  string str;

  //output final map
  str = GetBaseFilename() + ".map";
  ofstream osMap(str.c_str());
  GetMPProblem()->WriteRoadmapForVizmo(osMap);
  osMap.close();

  //output stats
  str = GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << endl;
  stats->PrintAllStats(osStat, GetMPProblem()->GetRoadmap());
  stats->PrintClock("RRT Generation", osStat);
  osStat.close();

  if(m_debug) cout<<"\nEnd Finalizing BasicRRTStrategy"<<endl;
}


CfgType
BasicRRTStrategy::GoalBiasedDirection(){
  // Determine direction, make sure goal not found
  if (m_goalsNotFound.size() == 0){
    return CfgType(); 
  }
  else {
    size_t goalNum = LRand()%m_goalsNotFound.size();
    return m_goals[m_goalsNotFound[goalNum]];
  }
}

CfgType 
BasicRRTStrategy::SelectDirection(){
  Environment* env = GetMPProblem()->GetRoadmap()->GetEnvironment();
  CfgType dir;
  dir.GetRandomCfg(env);
  return dir;
}

BasicRRTStrategy::VID 
BasicRRTStrategy::ExpandTree(CfgType& _dir){
  // Setup MP Variables
  Environment* env = GetMPProblem()->GetEnvironment();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
  VID recentVID = INVALID_VID;
  CDInfo  cdInfo;
  // Find closest Cfg in map
  vector<VID> kClosest;
  vector<CfgType> cfgs;

  nf->GetMethod(m_nf)->KClosest(GetMPProblem()->GetRoadmap(), _dir, 1, back_inserter(kClosest));     
  CfgType nearest = GetMPProblem()->GetRoadmap()->m_pRoadmap->find_vertex(kClosest[0])->property();
  CfgType newCfg;
  int weight;

  if(!RRTExpand(GetMPProblem(), m_vc, m_dm, nearest, _dir, newCfg, m_delta, weight, cdInfo)) {
    if(m_debug) cout << "RRT could not expand!" << endl; 
    return recentVID;
  }
  // If good to go, add to roadmap
  if(dm->Distance(env, newCfg, nearest) >= m_minDist) {
    recentVID = GetMPProblem()->GetRoadmap()->m_pRoadmap->AddVertex(newCfg);
    //TODO fix weight
    pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
    GetMPProblem()->GetRoadmap()->m_pRoadmap->AddEdge(nearest, newCfg, weights);
    GetMPProblem()->GetRoadmap()->m_pRoadmap->find_vertex(recentVID)->property().SetStat("Parent", kClosest[0]);
  } 
  return recentVID;
}

void 
BasicRRTStrategy::ConnectTrees(VID _recentlyGrown) {
  //Setup MP variables
  Environment* env = GetMPProblem()->GetEnvironment();
  StatClass* stats = GetMPProblem()->GetStatClass();
  Roadmap<CfgType, WeightType>* rdmp = GetMPProblem()->GetRoadmap();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
  shared_ptr<LocalPlannerMethod<CfgType, WeightType> > lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_lp);
  LPOutput<CfgType, WeightType> lpOutput;

  stringstream clockName; clockName << "Iteration " << m_currentIteration << ", Component Connection";
  stats->StartClock(clockName.str());

  stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
  vector< pair<size_t,VID> > ccs;
  get_cc_stats(*(rdmp->m_pRoadmap),cmap, ccs);
  cmap.reset();
  if(ccs.size()==1) return;

  vector<VID> vidCC;
  vidCC.push_back(_recentlyGrown);
  CfgType c1 = rdmp->m_pRoadmap->find_vertex(_recentlyGrown)->property();
  typedef vector<pair<size_t, VID> >::iterator CCSIT;
  vector<VID> closestNodesOtherCCs;

  //find closest VID from other CCS
  for(CCSIT ccsit = ccs.begin(); ccsit!=ccs.end(); ccsit++){
    vector<VID> cc;
    get_cc(*(rdmp->m_pRoadmap),cmap,ccsit->second,cc);
    cmap.reset();
    vector<VID> closest;
    nf->GetMethod(m_nf)->KClosest(rdmp, cc.begin(), cc.end(), _recentlyGrown, 1, back_inserter(closest));
    if (closest.size() != 0) 
      closestNodesOtherCCs.push_back(closest[0]);
  }

  //find closest VID from other CCS reps
  vector<VID> closestNode;
  nf->GetMethod(m_nf)->KClosest(rdmp, closestNodesOtherCCs.begin(), 
      closestNodesOtherCCs.end(), _recentlyGrown, 1, back_inserter(closestNode));

  //attempt connection
  CfgType c2 = rdmp->m_pRoadmap->find_vertex(closestNode[0])->property();
  if(m_debug) cout << "Attempt Connection " << c1 << "\t" << c2 << endl;
  CfgType col;
  if(dm->Distance(env, c1, c2)<m_delta &&
      lp->IsConnected(env, *stats, dm, c1, c2, col, &lpOutput, env->GetPositionRes(), env->GetOrientationRes())){

    if(m_debug) cout << "Connected" << endl;
    rdmp->m_pRoadmap->AddEdge(_recentlyGrown, closestNode[0], lpOutput.edge);
  }
}

void
BasicRRTStrategy::EvaluateGoals(){
  // Setup MP Variables
  StatClass* stats = GetMPProblem()->GetStatClass();
  Environment* env = GetMPProblem()->GetRoadmap()->GetEnvironment();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  LocalPlanners<CfgType,WeightType>* lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners();
  NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
  LPOutput<CfgType,WeightType> lpOutput;
  // Check if goals have been found
  for(vector<size_t>::iterator i = m_goalsNotFound.begin(); i!=m_goalsNotFound.end(); i++){
    vector<VID> closests;
    nf->GetMethod(m_nf)->KClosest(GetMPProblem()->GetRoadmap(), m_goals[*i], 1, back_inserter(closests));     
    CfgType closest = GetMPProblem()->GetRoadmap()->m_pRoadmap->find_vertex(closests[0])->property();
    double dist = dm->Distance(env, m_goals[*i], closest);
    if(m_debug) cout << "Distance to goal::" << dist << endl;
    CfgType col;
    if(dist < m_delta && lp->GetMethod(m_lp)->IsConnected(env, *stats, dm, closest, m_goals[*i], col, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true, false, false)){
      if(m_debug) cout << "Goal found::" << m_goals[*i] << endl;
      GetMPProblem()->GetRoadmap()->m_pRoadmap->AddVertex(m_goals[*i]);
      GetMPProblem()->GetRoadmap()->m_pRoadmap->AddEdge(closest, m_goals[*i], lpOutput.edge);
      m_goalsNotFound.erase(i);
      i--;
    }
  }
}

RoadmapClearanceStats 
BasicRRTStrategy::PathClearance(){
  RoadmapGraph<CfgType, WeightType>* graph = GetMPProblem()->GetRoadmap()->m_pRoadmap;
  int svid = graph->GetVID(m_roots[0]);
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
    double currentClearance = MinEdgeClearance(GetMPProblem(), false, GetMPProblem()->GetRoadmap()->GetEnvironment(), (*graph->find_vertex((*ei).source())).property(), (*graph->find_vertex((*ei).target())).property(), weight, m_vc, m_dm); 
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

}


