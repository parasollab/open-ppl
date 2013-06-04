/////////////////////////
//Class BasicRRTStrategy
////////////////////////

#ifndef BASICRRTSTRATEGY_H_
#define BASICRRTSTRATEGY_H_

#include "MPStrategyMethod.h"
#include "Utilities/RRTExpand.h"
#include <boost/make_shared.hpp>

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

    //Non-XML constructor sets all private variables
    BasicRRTStrategy(string _lp="sl", string _dm="euclidean",
        string _nf="bfnf", string _vc="cd1", string _nc="kClosest", string _gt="UNDIRECTED_TREE", vector<string> _evaluators=vector<string>(),
        double _delta=10.0, double _minDist=0.01, double _growthFocus=0.05, bool _evaluateGoal=true, CfgType _start=CfgType(), CfgType _goal=CfgType(),
        size_t _numRoots=1, size_t _numDirections=1, bool _growGoals=false);

    BasicRRTStrategy(MPProblemType* _problem, XMLNodeReader& _node, bool _warnXML = true);

    virtual ~BasicRRTStrategy();

    virtual void ParseXML(XMLNodeReader& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void PrintOptions(ostream& _os);

    void SetMPProblem(MPProblemType* _problem);

    //return path computed if RRT is successfully run with a query
    vector<CfgType> GetPath(){return m_query->GetPath();}

  protected:
    // Helper functions
    CfgType GoalBiasedDirection();
    CfgType SelectDirection();
    virtual VID ExpandTree(CfgType& _dir);
    void ConnectTrees(VID _recentlyGrown);
    void ConnectNeighbors(VID _newVID, VID _nearVID);
    void EvaluateGoals();

    vector<string> m_evaluators;
    string m_lp;
    string m_dm;
    string m_nf;
    string m_vc;
    shared_ptr<Query<MPTraits> > m_query;
    string m_nc;
    string m_gt;
    double m_delta, m_minDist, m_growthFocus;
    bool m_evaluateGoal;
    size_t m_numRoots,m_numDirections;
    bool m_growGoals;
    vector< vector<VID> > m_trees;
    typename vector<vector<VID> >::iterator m_currentTree;
    vector<CfgType> m_goals, m_roots;
    vector<size_t> m_goalsNotFound;
};

template<class MPTraits>
BasicRRTStrategy<MPTraits>::BasicRRTStrategy(string _lp, string _dm,
    string _nf, string _vc, string _nc, string _gt, vector<string> _evaluators,
    double _delta, double _minDist, double _growthFocus, bool _evaluateGoal, CfgType _start, CfgType _goal,
    size_t _numRoots, size_t _numDirections, bool _growGoals):
  m_evaluators(_evaluators), m_lp(_lp), m_dm(_dm), m_nf(_nf), m_vc(_vc), m_query(new Query<MPTraits>(_start, _goal)),
  m_nc(_nc), m_gt(_gt),  m_delta(_delta), m_minDist(_minDist), m_growthFocus(_growthFocus),
  m_evaluateGoal(_evaluateGoal), m_numRoots(_numRoots), m_numDirections(_numDirections), m_growGoals(_growGoals){
    this->SetName("BasicRRTStrategy");
  }

template<class MPTraits>
BasicRRTStrategy<MPTraits>::BasicRRTStrategy(MPProblemType* _problem, XMLNodeReader& _node, bool _warnXML) :
  MPStrategyMethod<MPTraits>(_problem, _node){
    this->SetName("BasicRRTStrategy");
    ParseXML(_node);
    if (_warnXML) _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
BasicRRTStrategy<MPTraits>::~BasicRRTStrategy(){ }

template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::ParseXML(XMLNodeReader& _node) {
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr){
    if(citr->getName() == "Evaluator"){
      string evalMethod = citr->stringXMLParameter("label", true, "", "Evaluation Method");
      m_evaluators.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    } 
    else
      citr->warnUnknownNode();
  }

  m_delta = _node.numberXMLParameter("delta", false, 1.0, 0.0, MAX_DBL, "Delta Distance");
  m_minDist = _node.numberXMLParameter("minDist", false, 0.0, 0.0, m_delta, "Minimum Distance");
  m_numRoots = _node.numberXMLParameter("numRoots", false, 1, 0, MAX_INT, "Number of Roots");
  m_growthFocus = _node.numberXMLParameter("growthFocus", false, 0.0, 0.0, 1.0, "#GeneratedTowardsGoal/#Generated");
  m_vc = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
  m_nf = _node.stringXMLParameter("nfLabel", true, "", "Neighborhood Finder");
  m_dm = _node.stringXMLParameter("dmLabel",true,"","Distance Metric");
  m_lp = _node.stringXMLParameter("lpLabel", true, "", "Local Planning Method");
  m_numDirections = _node.numberXMLParameter("m", false, 1, 1, 1000, "Number of directions to extend");
  m_nc = _node.stringXMLParameter("connectorLabel",false,"","Node Connection Method");
  m_gt = _node.stringXMLParameter("gtype",true,"","Graph type dir/undirected tree/graph");
  m_evaluateGoal = _node.boolXMLParameter("evaluateGoal", false, false, "");
  m_growGoals = _node.boolXMLParameter("growGoals",false,false,"Determines whether or not we grow a tree from the goal");

  //optionally read in a query and create a Query object.
  string query = _node.stringXMLParameter("query", false, "", "Query Filename");
  if(query!=""){
    m_query = shared_ptr<Query<MPTraits> >(new Query<MPTraits>(query));
    m_query->SetMPProblem(this->GetMPProblem());
    m_query->SetDebug(this->m_debug);
  }
}

template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::PrintOptions(ostream& _os) {
  typedef vector<string>::iterator SIT;
  _os << "BasicRRTStrategy::PrintOptions" << endl;
  _os << "\tNeighborhood Finder:: " << m_nf << endl;
  _os << "\tDistance Metric:: " << m_dm << endl;
  _os << "\tValidity Checker:: " << m_vc << endl;
  _os << "\tLocal Planner:: " << m_lp << endl;
  _os << "\tConnection Method:: " << m_nc << endl;
  _os << "\tGraph Type:: " << m_gt << endl;
  _os << "\tEvaluate Goal:: " << m_evaluateGoal << endl;
  _os << "\tEvaluators:: " << endl;
  _os << "\tGrow Goals:: " << m_growGoals << endl;
  for(SIT sit = m_evaluators.begin(); sit!=m_evaluators.end(); sit++)
    _os << "\t\t" << *sit << endl;
  _os << "\tdelta:: " << m_delta << endl;
  _os << "\tminimum distance:: " << m_minDist << endl;
  _os << "\tnumber of roots:: " << m_numRoots << endl;
  _os << "\tgrowth focus:: " << m_growthFocus << endl;
  _os << "\tnumber of expansion directions:: " << m_numDirections << endl;
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
  if(m_query){
    vector<CfgType>& queryCfgs = m_query->GetQuery();
    typedef typename vector<CfgType>::iterator CIT;
    if(m_growGoals){
      for(CIT cit1 = queryCfgs.begin();cit1 != queryCfgs.end();cit1++){
        VID add = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(*cit1);
        m_trees.push_back(vector<VID>(1,add));
      }
    }
    else{
      for(CIT cit1 = queryCfgs.begin(), cit2 = cit1+1; cit2!=queryCfgs.end(); cit1++, cit2++){
        VID add = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(*cit1);
        m_trees.push_back(vector<VID>(1,add));
        m_goals.push_back(*cit2);
        m_goalsNotFound.push_back(m_goals.size()-1);
      }
    }
  }
  else{
    // Add root vertex/vertices
    size_t i = 0;
    while(i < m_numRoots){
      tmp.GetRandomCfg(env);
      if(env->InBounds(tmp)
          && this->GetMPProblem()->GetValidityChecker(m_vc)->IsValid(tmp, env, *stats, cdInfo, &callee)){ 
        VID add = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(tmp);
        m_trees.push_back(vector<VID>(1,add));
      }
    }
  }

  m_currentTree = m_trees.begin();//set initial tree to be grown

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
    if(!m_growGoals){ 
      mapPassedEvaluation = m_trees.size()==1 && evalMap && ((m_evaluateGoal && m_goalsNotFound.size()==0) || !m_evaluateGoal);
      if(this->m_debug && m_goalsNotFound.size()==0)
        cout << "RRT FOUND ALL GOALS" << endl;
    }
    else mapPassedEvaluation = evalMap && m_trees.size()==1;
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
  if(m_query){
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
    vector<VID> currentVID;
    currentVID.push_back(_newVID);

    ConnectorPointer pConnection;
    pConnection = this->GetMPProblem()->GetConnector(m_nc);
    stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cmap;

    StatClass* conStatClass = this->GetMPProblem()->GetStatClass();
    string conClockName = "Total Connection time ";
    conStatClass->StartClock(conClockName);

    pConnection->Connect(this->GetMPProblem()->GetRoadmap(),
          *(this->GetMPProblem()->GetStatClass()), cmap,
          currentVID.begin(), currentVID.end(),
          m_currentTree->begin(), m_currentTree->end());
    
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
  nf->KClosest(this->GetMPProblem()->GetRoadmap(), m_currentTree->begin(), m_currentTree->end(), _dir, 1, back_inserter(kClosest));
  kcloseStatClass->StopClock(kcloseClockName);

  CfgType nearest = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(kClosest[0]);
  CfgType newCfg;
  int weight;

  StatClass* expandStatClass = this->GetMPProblem()->GetStatClass();
  string expandClockName = "RRTExpand time ";
  expandStatClass->StartClock(expandClockName);

  if(!RRTExpand<MPTraits>(this->GetMPProblem(), m_vc, m_dm, nearest, _dir, newCfg, m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
    if(this->m_debug) cout << "RRT could not expand!" << endl; 
    return recentVID;
  }

  expandStatClass->StopClock(expandClockName);

  if(this->m_debug) cout << "RRT expanded to " << newCfg << endl;

  // If good to go, add to roadmap
  if(dm->Distance(env, newCfg, nearest) >= m_minDist ) {
    recentVID = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(newCfg);
    m_currentTree->push_back(recentVID);
    if(std::string::npos!=m_gt.find("UNDIRECTED")){
      pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
      this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(kClosest[0], recentVID, weights);
    }
    else
      this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(kClosest[0], recentVID, WeightType("RRTExpand", weight));
    this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(recentVID).SetStat("Parent", kClosest[0]);


    if(std::string::npos!=m_gt.find("GRAPH")){
      ConnectNeighbors(recentVID, kClosest[0]);
    }

    for( size_t i=2 ;i<=m_numDirections; i++){//expansion to other m-1 directions
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
        VID otherVID = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(newCfg);
        m_currentTree->push_back(otherVID);
        if(std::string::npos!=m_gt.find("UNDIRECTED")){
          pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
          this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(kClosest[0], otherVID, weights);
        }
        else 
          this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(kClosest[0], otherVID, WeightType("RRTExpand", weight));        
        this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(otherVID).SetStat("Parent", kClosest[0]);

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
BasicRRTStrategy<MPTraits>::ConnectTrees(VID _recentlyGrown){
  //Setup MP variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dm);
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nf);
  CDInfo  cdInfo;
  stringstream clockName; clockName << "Component Connection";
  stats->StartClock(clockName.str());

  if(m_trees.size()==1) return; //trees connected

  CfgType c1 = rdmp->GetGraph()->GetVertex(_recentlyGrown);

  VID closestNode;
  CfgType closestCfg;
  double minDis = MAX_DBL;
  typename vector<vector<VID> >::iterator treeClosest = m_currentTree;
  CfgType c2;  
  typedef typename vector<vector<VID> >::iterator TRIT;

  for(TRIT trit = m_trees.begin(); trit!=m_trees.end(); ++trit){
    if(trit != m_currentTree){
      vector<VID> closest;
      nf->KClosest(rdmp, trit->begin(), trit->end(), _recentlyGrown, 1, back_inserter(closest));
      if (closest.size() != 0){
        c2  = rdmp->GetGraph()->GetVertex(closest[0]);
        double dist = dm->Distance(env,c1,c2);
        if(dist<minDis){
          treeClosest = trit;
          minDis = dist;
          closestNode = closest[0];
          closestCfg = c2;
        }
      }
    }
  }

  CfgType newCfg;
  int weight;
  if(!RRTExpand<MPTraits>(this->GetMPProblem(), m_vc, m_dm, closestCfg, c1, newCfg, MAX_DBL, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
    if(this->m_debug) cout << "RRT could not expand in connection phase, trapped!" << endl;
    return;
  }
  else{//add node to roadmap provided it's faraway enough or connects two trees
    VID newVID;
    if(c1 == newCfg){//we connect the trees
      if(this->m_debug) cout<<"We connected the trees"<<endl;
      if(distance(m_trees.begin(), m_currentTree) > distance(m_trees.begin(), treeClosest)){
        swap(m_currentTree, treeClosest);
      }
      m_currentTree->insert(m_currentTree->end(),treeClosest->begin(), treeClosest->end());
      m_trees.erase(treeClosest);
      newVID = _recentlyGrown;
    }
    else if(dm->Distance(env, newCfg, c2) >= m_minDist ) {
      newVID = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(newCfg);
      treeClosest->push_back(newVID);
      m_currentTree = treeClosest;
    }
    else  //too close to add, keep expanding current tree
      return;

    //add to roadmap 
    if(std::string::npos!=m_gt.find("UNDIRECTED")){
      pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
      this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(closestNode, newVID, weights);
    }
    else
      this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(closestNode, newVID, WeightType("RRTExpand", weight));
    this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(closestNode).SetStat("Parent", newVID);

    if(std::string::npos!=m_gt.find("GRAPH")){
      ConnectNeighbors(newVID, closestNode);
    }
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
  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();

  LPOutput<MPTraits> lpOutput;
  // Check if goals have been found
  for(vector<size_t>::iterator i = m_goalsNotFound.begin(); i!=m_goalsNotFound.end(); i++){
    vector<VID> closests;
    nfp->KClosest(rdmp, m_goals[*i], 1, back_inserter(closests));     
    CfgType closest = rdmp->GetGraph()->GetVertex(closests[0]);
    double dist = dmp->Distance(env, m_goals[*i], closest);
    if(this->m_debug) cout << "Distance to goal::" << dist << endl;
    CfgType col;
    if(dist < m_delta && lpp->IsConnected(env, *stats, dmp, closest, m_goals[*i], col, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true, false, false)){
      if(this->m_debug) cout << "Goal found::" << m_goals[*i] << endl;
      VID goalVID;
      if(!(rdmp->GetGraph()->IsVertex( m_goals[*i])))
        goalVID = rdmp->GetGraph()->AddVertex(m_goals[*i]);
      else
        goalVID = rdmp->GetGraph()->GetVID(m_goals[*i]);
      rdmp->GetGraph()->AddEdge(closests[0], goalVID, lpOutput.edge);
      m_goalsNotFound.erase(i);
      i--;
    }
  }
}

template<class MPTraits>
void 
BasicRRTStrategy<MPTraits>::SetMPProblem(MPProblemType* _problem){
  MPBaseObject<MPTraits>::SetMPProblem(_problem);
  if(m_query)
    m_query->SetMPProblem(_problem);
}
#endif
