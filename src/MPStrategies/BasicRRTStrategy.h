/////////////////////////
//Class BasicRRTStrategy
////////////////////////

#ifndef BASICRRTSTRATEGY_H_
#define BASICRRTSTRATEGY_H_

#include "MPStrategyMethod.h"
#include "graph/algorithms/count_hop_pairs.h"
/*#include "IOUtils.h"
#include "MetricUtils.h"*/

template<class MPTraits>
class BasicRRTStrategy : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    
    BasicRRTStrategy();
    BasicRRTStrategy(MPProblemType* _problem, XMLNodeReader& _node, bool _warnXML = true);
    virtual ~BasicRRTStrategy();
    
    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void PrintOptions(ostream& _os);

  protected:
    // Helper functions
    CfgType GoalBiasedDirection();
    CfgType SelectDirection();
    virtual VID ExpandTree(CfgType& _dir);
    void ConnectTrees(VID _recentlyGrown);
    void ConnectNeighbors(VID _newVID, VID _nearVID);
    void EvaluateGoals();
    
    vector<string> m_evaluators;
    string m_sampler;
    string m_lp;
    string m_dm;
    string m_nf;
    string m_vc;
    Query<MPTraits>* m_query;
    string m_nc;
    string m_gt;
    double m_delta, m_minDist, m_growthFocus;
    bool m_evaluateGoal;
    int m_numRoots;
    int m_m,m_h;
    vector<CfgType> m_goals, m_roots;
    vector<size_t> m_goalsNotFound;
};

template<class MPTraits>
BasicRRTStrategy<MPTraits>::BasicRRTStrategy(): m_query(NULL){
  this->SetName("BasicRRTStrategy");
}

template<class MPTraits>
BasicRRTStrategy<MPTraits>::BasicRRTStrategy(MPProblemType* _problem, XMLNodeReader& _node, bool _warnXML) :
  MPStrategyMethod<MPTraits>(_problem, _node), m_query(NULL){
    this->SetName("BasicRRTStrategy");
    ParseXML(_node);
    if (_warnXML) _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
BasicRRTStrategy<MPTraits>::~BasicRRTStrategy(){
  if(m_query != NULL)
    delete m_query;
}

template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::ParseXML(XMLNodeReader& _node) {
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
    m_query = new Query<MPTraits>(query);
    m_query->SetMPProblem(this->GetMPProblem());
    m_query->SetDebug(this->m_debug);
  }
}

template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::PrintOptions(ostream& _os) {
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
template<class MPTraits>
void 
BasicRRTStrategy<MPTraits>::Initialize(){
  if(this->m_debug) cout<<"\nInitializing BasicRRTStrategy::"<<endl;

  // Setup MP variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CDInfo cdInfo;
  string callee = "BasicRRTStrategy::RRT";
  // Setup RRT Variables
  CfgType tmp;
  if(m_query != NULL){
    vector<CfgType>& queryCfgs = m_query->GetQuery();
    typedef typename vector<CfgType>::iterator CIT;
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
          && this->GetMPProblem()->GetValidityChecker(m_vc)->IsValid(tmp, env, *stats, cdInfo, &callee)){
        m_roots.push_back(tmp);
        m_goals.push_back(tmp);
        m_goalsNotFound.push_back(i++);
      }
    }
  }
  
  for(typename vector<CfgType>::iterator C = m_roots.begin(); C!=m_roots.end(); C++){
    this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(*C);
  }

  if(this->m_debug) cout<<"\nEnding Initializing BasicRRTStrategy"<<endl;
}

////////////////
//Run/Start Phase
////////////////
template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::Run() {
  if(this->m_debug) cout << "\nRunning BasicRRTStrategy::" << endl;

  // Setup MP Variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();

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
    bool evalMap = this->EvaluateMap(m_evaluators);
    mapPassedEvaluation = evalMap && ((m_evaluateGoal && m_goalsNotFound.size()==0) || !m_evaluateGoal);

    if(this->m_debug && m_goalsNotFound.size()==0)
      cout << "RRT FOUND ALL GOALS" << endl;
  }

  stats->StopClock("RRT Generation");
  if(this->m_debug) {
    stats->PrintClock("RRT Generation", cout);
    cout<<"\nEnd Running BasicRRTStrategy::" << endl;  
  }
}

/////////////////////
//Finalization phase
////////////////////
template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::Finalize() {
 
  if(this->m_debug) cout<<"\nFinalizing BasicRRTStrategy::"<<endl;

  //setup variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  string str;

  //perform query if query was given as input
  if(m_query != NULL){
    str = this->GetBaseFilename() + ".path";
    m_query->SetPathFile(str);
    if(m_evaluateGoal){
      if(m_query->PerformQuery(this->GetMPProblem()->GetRoadmap(), *stats)){
        if(this->m_debug) cout << "Query successful! Output written to " << str << "." << endl;
      }
      else{
        if(this->m_debug) cout << "Query unsuccessful." << endl;
      }
    }
  }

  //output final map
  str = this->GetBaseFilename() + ".map";
  ofstream osMap(str.c_str());
  this->GetMPProblem()->GetRoadmap()->Write(osMap, this->GetMPProblem()->GetEnvironment());
  osMap.close();

  //output stats
  str = this->GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << endl;
  stats->PrintAllStats(osStat, this->GetMPProblem()->GetRoadmap());
  stats->PrintClock("RRT Generation", osStat);
  osStat.close();

  if(this->m_debug) cout<<"\nEnd Finalizing BasicRRTStrategy"<<endl;
}

template<class MPTraits>
typename MPTraits::CfgType
BasicRRTStrategy<MPTraits>::GoalBiasedDirection(){
  // Determine direction, make sure goal not found
  if (m_goalsNotFound.size() == 0){
    return CfgType(); 
  }
  else {
    size_t goalNum = LRand()%m_goalsNotFound.size();
    return m_goals[m_goalsNotFound[goalNum]];
  }
}

template<class MPTraits>
typename MPTraits::CfgType 
BasicRRTStrategy<MPTraits>::SelectDirection(){
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CfgType dir;
  dir.GetRandomCfg(env);
  return dir;
}

template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::ConnectNeighbors(VID _newVID, VID _nearVID){
  if (_newVID != INVALID_VID) {
    vector<VID> allVIDs;
    vector<VID> currentVID;
    currentVID.push_back(_newVID);
    vector <VID> vRes;
    if( m_h>0){
      StatClass * hopStatClass = this->GetMPProblem()->GetStatClass();
      string hopClockName = "Total bfs time ";
      hopStatClass->StartClock(hopClockName);

      stapl::sequential::map_property_map<typename GraphType::GRAPH, size_t> hopmap;
      hopmap.put(_nearVID, 0);
      stapl::sequential::map_property_map<typename GraphType::GRAPH, size_t> colormap;
      RoadmapGraph<CfgType,WeightType>* rmapG=this->GetMPProblem()->GetRoadmap()->GetGraph();
      stapl::sequential::hops_detail::hops_visitor<typename GraphType::GRAPH> vis(*rmapG,hopmap,m_h);
      breadth_first_search_early_quit(*rmapG,_nearVID,vis,colormap);
      vRes= vis.get_res();
      hopStatClass->StopClock(hopClockName);
    }
    for(typename GraphType::GRAPH::const_vertex_iterator vi = this->GetMPProblem()->GetRoadmap()->GetGraph()->begin(); vi!= this->GetMPProblem()->GetRoadmap()->GetGraph()->end(); vi++){
        allVIDs.push_back((*vi).descriptor());
    }
    ConnectorPointer pConnection;
    pConnection = this->GetMPProblem()->GetConnector(m_nc);
    stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cmap;

    StatClass* conStatClass = this->GetMPProblem()->GetStatClass();
    string conClockName = "Total Connection time ";
    conStatClass->StartClock(conClockName);

    if(m_h==0) //full roadmap
      pConnection->Connect(this->GetMPProblem()->GetRoadmap(),
        *(this->GetMPProblem()->GetStatClass()), cmap,
        currentVID.begin(), currentVID.end(),
        allVIDs.begin(), allVIDs.end());
    else {
      pConnection->Connect(this->GetMPProblem()->GetRoadmap(),
        *(this->GetMPProblem()->GetStatClass()), cmap,
        currentVID.begin(), currentVID.end(),
        vRes.begin(), vRes.end());
    }
    conStatClass->StopClock(conClockName);
  }
}

template<class MPTraits>
typename BasicRRTStrategy<MPTraits>::VID 
BasicRRTStrategy<MPTraits>::ExpandTree(CfgType& _dir){
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dm);
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nf);
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(m_lp);
  LPOutput<MPTraits> lpOutput;
  VID recentVID = INVALID_VID;
  CDInfo  cdInfo;
  // Find closest Cfg in map
  vector<VID> kClosest;
  vector<CfgType> cfgs;

  StatClass* kcloseStatClass = this->GetMPProblem()->GetStatClass();
  string kcloseClockName = "kclosest time ";
  kcloseStatClass->StartClock(kcloseClockName);
  nf->KClosest(this->GetMPProblem()->GetRoadmap(), _dir, 1, back_inserter(kClosest));
  kcloseStatClass->StopClock(kcloseClockName);
  

  CfgType nearest = this->GetMPProblem()->GetRoadmap()->GetGraph()->find_vertex(kClosest[0])->property();
  CfgType newCfg;
  int weight;

  StatClass* expandStatClass = this->GetMPProblem()->GetStatClass();
  string expandClockName = "RRTExpand time ";
  expandStatClass->StartClock(expandClockName);

  if(!RRTExpand<MPTraits>(this->GetMPProblem(), m_vc, m_dm, nearest, _dir, newCfg, m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
    if(this->m_debug) cout << "RRT could not expand!" << endl; 
    return recentVID;
  }
  cout<<"RRT expanded"<<endl;
  expandStatClass->StopClock(expandClockName);
  // If good to go, add to roadmap
  if(dm->Distance(env, newCfg, nearest) >= m_minDist ) {
    recentVID = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(newCfg);
    if(std::string::npos!=m_gt.find("UNDIRECTED")){
      pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
      this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(nearest, newCfg, weights);
    }
    else
      this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(nearest, newCfg, WeightType("RRTExpand", weight));
    this->GetMPProblem()->GetRoadmap()->GetGraph()->find_vertex(recentVID)->property().SetStat("Parent", kClosest[0]);

    if(std::string::npos!=m_gt.find("GRAPH")){
      VID newCfgVID = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVID(newCfg);
      ConnectNeighbors(newCfgVID, kClosest[0]);
    }

    for( int i=2 ;i<=m_m; i++){//expansion to other m-1 directions
      CfgType randdir = this->SelectDirection();
      expandStatClass->StartClock(expandClockName);
      bool expandFlag = RRTExpand<MPTraits>(this->GetMPProblem(), m_vc, m_dm, nearest, randdir, newCfg, m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes());

      expandStatClass->StopClock(expandClockName);
      StatClass* conStatClass = this->GetMPProblem()->GetStatClass();
      string conClockName = "Connection time ";
      conStatClass->StartClock(conClockName);
    
      if(!expandFlag) {
        if(this->m_debug) cout << "RRT could not expand to additional directions!" << endl;
      }  
      else if(dm->Distance(env, newCfg, nearest) >= m_minDist ) {
        VID otherVID;
        otherVID = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(newCfg);
        if(std::string::npos!=m_gt.find("UNDIRECTED")){
          pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
          this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(nearest, newCfg, weights);
        }
        else
          this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(nearest, newCfg, WeightType("RRTExpand", weight));        

        this->GetMPProblem()->GetRoadmap()->GetGraph()->find_vertex(otherVID)->property().SetStat("Parent", kClosest[0]);
        if(std::string::npos!=m_gt.find("GRAPH")){
          ConnectNeighbors( otherVID, kClosest[0]);
        }
      }
      conStatClass->StopClock(conClockName);
    }
  }
 
 return recentVID;
}

template<class MPTraits>
void 
BasicRRTStrategy<MPTraits>::ConnectTrees(VID _recentlyGrown) {
  //Setup MP variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dm);
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nf);
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(m_lp);
  LPOutput<MPTraits> lpOutput;

  stringstream clockName; clockName << "Component Connection";
  stats->StartClock(clockName.str());

  stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cmap;
  vector< pair<size_t,VID> > ccs;
  stapl::sequential::get_cc_stats(*(rdmp->GetGraph()),cmap, ccs);
  cmap.reset();
  if(ccs.size()==1) return;

  vector<VID> vidCC;
  vidCC.push_back(_recentlyGrown);
  CfgType c1 = rdmp->GetGraph()->find_vertex(_recentlyGrown)->property();
  typedef typename vector<pair<size_t, VID> >::iterator CCSIT;
  vector<VID> closestNodesOtherCCs;

  //find closest VID from other CCS
  for(CCSIT ccsit = ccs.begin(); ccsit!=ccs.end(); ccsit++){
    vector<VID> cc;
    stapl::sequential::get_cc(*(rdmp->GetGraph()),cmap,ccsit->second,cc);
    cmap.reset();
    vector<VID> closest;
    nf->KClosest(rdmp, cc.begin(), cc.end(), _recentlyGrown, 1, back_inserter(closest));
    if (closest.size() != 0) 
      closestNodesOtherCCs.push_back(closest[0]);
  }

  //find closest VID from other CCS reps
  vector<VID> closestNode;
  nf->KClosest(rdmp, closestNodesOtherCCs.begin(), 
      closestNodesOtherCCs.end(), _recentlyGrown, 1, back_inserter(closestNode));

  //attempt connection
  CfgType c2 = rdmp->GetGraph()->find_vertex(closestNode[0])->property();
  if(this->m_debug) cout << "Attempt Connection " << c1 << "\t" << c2 << endl;
  CfgType col;
  if(dm->Distance(env, c1, c2)<m_delta &&
      lp->IsConnected(env, *stats, dm, c1, c2, col, &lpOutput, env->GetPositionRes(), env->GetOrientationRes())){

    if(this->m_debug) cout << "Connected" << endl;
    rdmp->GetGraph()->AddEdge(_recentlyGrown, closestNode[0], lpOutput.edge);
  }
}

template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::EvaluateGoals(){
  // Setup MP Variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dmp = this->GetMPProblem()->GetDistanceMetric(m_dm);
  LocalPlannerPointer lpp = this->GetMPProblem()->GetLocalPlanner(m_lp);
  NeighborhoodFinderPointer nfp = this->GetMPProblem()->GetNeighborhoodFinder(m_nf);
  LPOutput<MPTraits> lpOutput;
  // Check if goals have been found
  for(vector<size_t>::iterator i = m_goalsNotFound.begin(); i!=m_goalsNotFound.end(); i++){
    vector<VID> closests;
    nfp->KClosest(this->GetMPProblem()->GetRoadmap(), m_goals[*i], 1, back_inserter(closests));     
    CfgType closest = this->GetMPProblem()->GetRoadmap()->GetGraph()->find_vertex(closests[0])->property();
    double dist = dmp->Distance(env, m_goals[*i], closest);
    if(this->m_debug) cout << "Distance to goal::" << dist << endl;
    CfgType col;
    if(dist < m_delta && lpp->IsConnected(env, *stats, dmp, closest, m_goals[*i], col, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true, false, false)){
      if(this->m_debug) cout << "Goal found::" << m_goals[*i] << endl;
      if(!(this->GetMPProblem()->GetRoadmap()->GetGraph()->IsVertex( m_goals[*i])))
        this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(m_goals[*i]);
      this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(closest, m_goals[*i], lpOutput.edge);
      m_goalsNotFound.erase(i);
      i--;
    }
  }
}

#endif
