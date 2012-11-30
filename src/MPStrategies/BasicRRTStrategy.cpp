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
#include "GraphAlgo.h"
#include "graph/algorithms/count_hop_pairs.h"

///////////////////////
//Constructors
//////////////////////
BasicRRTStrategy::BasicRRTStrategy(XMLNodeReader& _node, MPProblem* _problem, bool _warnXML) :
  MPStrategyMethod(_node, _problem), m_query(NULL){
    ParseXML(_node);
    if (_warnXML) _node.warnUnrequestedAttributes();
    if(m_debug && _warnXML) PrintOptions(cout);
  }

BasicRRTStrategy::~BasicRRTStrategy(){
  if(m_query != NULL)
    delete m_query;
}

void
BasicRRTStrategy::ParseXML(XMLNodeReader& _node) {
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr){
    if(citr->getName() == "evaluation_method"){
      string evalMethod = citr->stringXMLParameter("Method", true, "", "Evaluation Method");
      m_evaluators.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    } 
    else
      citr->warnUnknownNode();
  }

  m_delta = _node.numberXMLParameter("delta", false, 1.0, 0.0, MAX_DBL, "Delta Distance");
  m_minDist = _node.numberXMLParameter("minDist", false, 0.0, 0.0, MAX_DBL, "Minimum Distance");
  m_numRoots = _node.numberXMLParameter("numRoots", false, 1, 0, MAX_INT, "Number of Roots");
  m_growthFocus = _node.numberXMLParameter("growthFocus", false, 0.0, 0.0, 1.0, "#GeneratedTowardsGoal/#Generated");
  m_h = _node.numberXMLParameter("h", false, 0, 1, MAX_INT, "Hop Limit");
  m_sampler = _node.stringXMLParameter("sampler", true, "", "Sampler Method");
  m_vc = _node.stringXMLParameter("vcMethod", true, "", "Validity Test Method");
  m_nf = _node.stringXMLParameter("nf", true, "", "Neighborhood Finder");
  m_dm = _node.stringXMLParameter("dmMethod",true,"","Distance Metric");
  m_lp = _node.stringXMLParameter("lp", true, "", "Local Planning Method");
  m_m= _node.numberXMLParameter("m", false, 1.0, 1.0, 1000.0, "Number of directions to extend");
  m_nc = _node.stringXMLParameter("connectionMethod",false,"","Node Connection Method");
  m_gt = _node.stringXMLParameter("gtype",true,"","Graph type dir/undirected tree/graph");
  m_evaluateGoal = _node.boolXMLParameter("evaluateGoal", false, false, "");

  //optionally read in a query and create a Query object.
  string query = _node.stringXMLParameter("query", false, "", "Query Filename");
  if(query != ""){
    m_query = new Query<CfgType, WeightType>(query);
    m_query->SetMPProblem(GetMPProblem());
    m_query->SetDebug(m_debug);
  }
}

void
BasicRRTStrategy::PrintOptions(ostream& _os) {
  typedef vector<string>::iterator SIT;
  _os << "BasicRRTStrategy::PrintOptions" << endl;
  _os << "\tSampler:: " << m_sampler << endl;
  _os << "\tNeighorhood Finder:: " << m_nf << endl;
  _os << "\tDistance Metric:: " << m_dm << endl;
  _os << "\tValidity Checker:: " << m_vc << endl;
  _os << "\tLocal Planner:: " << m_lp << endl;
  _os << "\tConnection Method:: " << m_nc << endl;
  _os << "\tGraph Type:: " << m_gt << endl;
  _os << "\tHop Limit:: " << m_h << endl;
  _os << "\tEvaluate Goal:: " << m_evaluateGoal << endl;
  _os << "\tEvaluators:: " << endl;
  for(SIT sit = m_evaluators.begin(); sit!=m_evaluators.end(); sit++)
    _os << "\t\t" << *sit << endl;
  _os << "\tdelta:: " << m_delta << endl;
  _os << "\tminimum distance:: " << m_minDist << endl;
  _os << "\tnumber of roots:: " << m_numRoots << endl;
  _os << "\tgrowth focus:: " << m_growthFocus << endl;
  _os << "\tnumber of expansion directions:: " << m_m << endl;
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
  if(m_query != NULL){
    vector<CfgType>& queryCfgs = m_query->GetQuery();
    typedef vector<CfgType>::iterator CIT;
    for(CIT cit1 = queryCfgs.begin(), cit2 = cit1+1; cit2!=queryCfgs.end(); cit1++, cit2++){
      m_roots.push_back(*cit1);
      m_goals.push_back(*cit2);
      m_goalsNotFound.push_back(m_goals.size()-1);
    }
  }
  else{
    // Add root vertex/vertices
    int i = 0;
    while(i < m_numRoots){
      tmp.GetRandomCfg(env);
      if (tmp.InBoundary(env)
          && vc->GetMethod(m_vc)->IsValid(tmp, env, *stats, cdInfo, &callee)){
        m_roots.push_back(tmp);
        m_goals.push_back(tmp);
        m_goalsNotFound.push_back(i++);
      }
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
void
BasicRRTStrategy::Run() {
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
      if(m_evaluateGoal)
        EvaluateGoals();
    }
    
    //evaluate the roadmap
    bool evalMap = EvaluateMap(m_evaluators);
    mapPassedEvaluation = evalMap && ((m_evaluateGoal && m_goalsNotFound.size()==0) || !m_evaluateGoal);

    if(m_debug && m_goalsNotFound.size()==0)
      cout << "RRT FOUND ALL GOALS" << endl;
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
void
BasicRRTStrategy::Finalize() {
 
  if(m_debug) cout<<"\nFinalizing BasicRRTStrategy::"<<endl;

  //setup variables
  StatClass* stats = GetMPProblem()->GetStatClass();
  string str;

  //perform query if query was given as input
  if(m_query != NULL){
    str = GetBaseFilename() + ".path";
    m_query->SetPathFile(str);
    if(m_evaluateGoal){
      if(m_query->PerformQuery(GetMPProblem()->GetRoadmap(), *stats)){
        if(m_debug) cout << "Query successful! Output written to " << str << "." << endl;
      }
      else{
        if(m_debug) cout << "Query unsuccessful." << endl;
      }
    }
  }

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

void
BasicRRTStrategy::ConnectNeighbors(VID _newVID, VID _nearVID){
  if (_newVID != INVALID_VID) {
    vector<VID> allVIDs;
    vector<VID> currentVID;
    currentVID.push_back(_newVID);
    vector <VID> vRes;
    if( m_h>0){
      StatClass * hopStatClass = GetMPProblem()->GetStatClass();
      string hopClockName = "Total bfs time ";
      hopStatClass->StartClock(hopClockName);

      stapl::sequential::map_property_map<GRAPH, size_t> hopmap;
      hopmap.put(_nearVID, 0);
      stapl::sequential::map_property_map<GRAPH, size_t> colormap;
      RoadmapGraph<CfgType,WeightType>* rmapG=GetMPProblem()->GetRoadmap()->m_pRoadmap;
      stapl::sequential::hops_detail::hops_visitor<GRAPH > vis(*rmapG,hopmap,m_h);
      breadth_first_search_early_quit(*rmapG,_nearVID,vis,colormap);
      vRes= vis.get_res();
      hopStatClass->StopClock(hopClockName);
    }
    for(GRAPH::const_vertex_iterator vi = GetMPProblem()->GetRoadmap()->m_pRoadmap->begin(); vi!= GetMPProblem()->GetRoadmap()->m_pRoadmap->end(); vi++){
        allVIDs.push_back((*vi).descriptor());
    }
    Connector<CfgType, WeightType>::ConnectionPointer pConnection;
    pConnection = GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(m_nc);
    stapl::sequential::vector_property_map< GRAPH,size_t > cmap;

    StatClass* conStatClass = GetMPProblem()->GetStatClass();
    string conClockName = "Total Connection time ";
    conStatClass->StartClock(conClockName);

    if(m_h==0) //full roadmap
      pConnection->Connect(GetMPProblem()->GetRoadmap(),
        *(GetMPProblem()->GetStatClass()), cmap,
        currentVID.begin(), currentVID.end(),
        allVIDs.begin(), allVIDs.end());
    else {
      pConnection->Connect(GetMPProblem()->GetRoadmap(),
        *(GetMPProblem()->GetStatClass()), cmap,
        currentVID.begin(), currentVID.end(),
        vRes.begin(), vRes.end());
    }
    conStatClass->StopClock(conClockName);
  }
}


BasicRRTStrategy::VID 
BasicRRTStrategy::ExpandTree(CfgType& _dir){
  // Setup MP Variables
  Environment* env = GetMPProblem()->GetEnvironment();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
  shared_ptr<LocalPlannerMethod<CfgType, WeightType> > lp = GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_lp);
  LPOutput<CfgType, WeightType> lpOutput;
  VID recentVID = INVALID_VID;
  CDInfo  cdInfo;
  // Find closest Cfg in map
  vector<VID> kClosest;
  vector<CfgType> cfgs;

  StatClass* kcloseStatClass = GetMPProblem()->GetStatClass();
  string kcloseClockName = "kclosest time ";
  kcloseStatClass->StartClock(kcloseClockName);
  nf->GetMethod(m_nf)->KClosest(GetMPProblem()->GetRoadmap(), _dir, 1, back_inserter(kClosest));
  kcloseStatClass->StopClock(kcloseClockName);
  

  CfgType nearest = GetMPProblem()->GetRoadmap()->m_pRoadmap->find_vertex(kClosest[0])->property();
  CfgType newCfg;
  int weight;

  StatClass* expandStatClass = GetMPProblem()->GetStatClass();
  string expandClockName = "RRTExpand time ";
  expandStatClass->StartClock(expandClockName);

  if(!RRTExpand(GetMPProblem(), m_vc, m_dm, nearest, _dir, newCfg, m_delta, weight, cdInfo)) {
    if(m_debug) cout << "RRT could not expand!" << endl; 
    return recentVID;
  }
  cout<<"RRT expanded"<<endl;
  expandStatClass->StopClock(expandClockName);
  // If good to go, add to roadmap
  if(dm->Distance(env, newCfg, nearest) >= m_minDist ) {
    recentVID = GetMPProblem()->GetRoadmap()->m_pRoadmap->AddVertex(newCfg);
    if(std::string::npos!=m_gt.find("UNDIRECTED")){
      pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
      GetMPProblem()->GetRoadmap()->m_pRoadmap->AddEdge(nearest, newCfg, weights);
    }
    else
      GetMPProblem()->GetRoadmap()->m_pRoadmap->AddEdge(nearest, newCfg, WeightType("RRTExpand", weight));
    GetMPProblem()->GetRoadmap()->m_pRoadmap->find_vertex(recentVID)->property().SetStat("Parent", kClosest[0]);

    if(std::string::npos!=m_gt.find("GRAPH")){
      VID newCfgVID = GetMPProblem()->GetRoadmap()->m_pRoadmap->GetVID(newCfg);
      ConnectNeighbors(newCfgVID, kClosest[0]);
    }

    for( int i=2 ;i<=m_m; i++){//expansion to other m-1 directions
      CfgType randdir =BasicRRTStrategy::SelectDirection();
      expandStatClass->StartClock(expandClockName);
      bool expandFlag = RRTExpand(GetMPProblem(), m_vc, m_dm, nearest, randdir, newCfg, m_delta, weight, cdInfo);

      expandStatClass->StopClock(expandClockName);
      StatClass* conStatClass = GetMPProblem()->GetStatClass();
      string conClockName = "Connection time ";
      conStatClass->StartClock(conClockName);
    
      if(!expandFlag) {
        if(m_debug) cout << "RRT could not expand to additional directions!" << endl;
      }  
      else if(dm->Distance(env, newCfg, nearest) >= m_minDist ) {
        VID otherVID;
        otherVID = GetMPProblem()->GetRoadmap()->m_pRoadmap->AddVertex(newCfg);
        if(std::string::npos!=m_gt.find("UNDIRECTED")){
          pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
          GetMPProblem()->GetRoadmap()->m_pRoadmap->AddEdge(nearest, newCfg, weights);
        }
        else
          GetMPProblem()->GetRoadmap()->m_pRoadmap->AddEdge(nearest, newCfg, WeightType("RRTExpand", weight));        

        GetMPProblem()->GetRoadmap()->m_pRoadmap->find_vertex(otherVID)->property().SetStat("Parent", kClosest[0]);
        if(std::string::npos!=m_gt.find("GRAPH")){
          ConnectNeighbors( otherVID, kClosest[0]);
        }
      }
      conStatClass->StopClock(conClockName);
    }
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

  stringstream clockName; clockName << "Component Connection";
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
      if(!(GetMPProblem()->GetRoadmap()->m_pRoadmap->IsVertex( m_goals[*i])))
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
    ClearanceParams cParams(GetMPProblem(), m_vc, m_dm, false);
    double currentClearance = MinEdgeClearance((*graph->find_vertex((*ei).source())).property(),
      (*graph->find_vertex((*ei).target())).property(), 
      weight, cParams); 
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


