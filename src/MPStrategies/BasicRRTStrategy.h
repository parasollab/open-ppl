/////////////////////////
//Class BasicRRTStrategy
////////////////////////

#ifndef BASICRRTSTRATEGY_H_
#define BASICRRTSTRATEGY_H_

#include "MPStrategyMethod.h"
#include "Extenders/BasicExtender.h"

template<class MPTraits>
class BasicRRTStrategy : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::CfgRef CfgRef;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::ExtenderPointer ExtenderPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;

    //Non-XML constructor sets all private variables
    BasicRRTStrategy(string _lp="sl", string _dm="euclidean",
        string _nf="bfnf", string _vc="cd1", string _nc="kClosest",
        string _gt="UNDIRECTED_TREE", string _extenderLabel="BERO",
        vector<string> _evaluators=vector<string>(),
        double _delta=10.0, double _minDist=0.01, double _growthFocus=0.05,
        bool _evaluateGoal=true, const CfgType& _start=CfgType(), const CfgType& _goal=CfgType(),
        size_t _numRoots=1, size_t _numDirections=1, size_t _maxTrial = 3, bool _growGoals=false);

    BasicRRTStrategy(MPProblemType* _problem, XMLNodeReader& _node, bool _warnXML = true, bool _child = false);

    virtual ~BasicRRTStrategy();

    virtual void ParseXML(XMLNodeReader& _node, bool _child = false);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

    void SetMPProblem(MPProblemType* _problem);

    //return path computed if RRT is successfully run with a query
    vector<CfgType> GetPath(){return m_query->GetPath();}

  protected:
    // Helper functions
    CfgType GoalBiasedDirection();
    CfgType SelectDirection();
    CfgType SelectDispersedDirection(VID v1);
    vector<CfgType> SelectNeighbors(VID v1);
    virtual VID ExpandTree(CfgType& _dir);
    void ConnectTrees(VID _recentlyGrown);
    void ConnectNeighbors(VID _newVID, VID _nearVID);
    void EvaluateGoals(VID _newVID);

    vector<string> m_evaluators;
    string m_lp;
    string m_dm;
    string m_nf;
    string m_vc;
    shared_ptr<Query<MPTraits> > m_query;
    string m_nc;
    string m_gt;
    string m_extenderLabel;
    double m_delta, m_minDist, m_growthFocus;
    bool m_evaluateGoal;
    size_t m_numRoots,m_numDirections, m_maxTrial;
    bool m_growGoals;
    vector< vector<VID> > m_trees;
    typename vector<vector<VID> >::iterator m_currentTree;
    vector<CfgType> m_goals;
    vector<size_t> m_goalsNotFound;

    ClearanceUtility<MPTraits> m_clearanceUtility;
};

template<class MPTraits>
BasicRRTStrategy<MPTraits>::BasicRRTStrategy(string _lp, string _dm,
    string _nf, string _vc, string _nc, string _gt, string _extenderLabel,
    vector<string> _evaluators, double _delta, double _minDist,
    double _growthFocus, bool _evaluateGoal, const CfgType& _start,
    const CfgType& _goal, size_t _numRoots, size_t _numDirections,
    size_t _maxTrial, bool _growGoals):
  m_evaluators(_evaluators), m_lp(_lp), m_dm(_dm), m_nf(_nf), m_vc(_vc),
  m_query(new Query<MPTraits>(_start, _goal)), m_nc(_nc), m_gt(_gt),
  m_extenderLabel(_extenderLabel), m_delta(_delta), m_minDist(_minDist),
  m_growthFocus(_growthFocus), m_evaluateGoal(_evaluateGoal),
  m_numRoots(_numRoots), m_numDirections(_numDirections), m_maxTrial(_maxTrial),
  m_growGoals(_growGoals) {
    this->SetName("BasicRRTStrategy");
  }

template<class MPTraits>
BasicRRTStrategy<MPTraits>::BasicRRTStrategy(MPProblemType* _problem, XMLNodeReader& _node, bool _warnXML, bool _child) :
  MPStrategyMethod<MPTraits>(_problem, _node), m_query((Query<MPTraits>*)NULL), m_clearanceUtility(_problem, _node){
    this->SetName("BasicRRTStrategy");
    ParseXML(_node, _child);
    if (_warnXML) _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
BasicRRTStrategy<MPTraits>::~BasicRRTStrategy(){ }

template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::ParseXML(XMLNodeReader& _node, bool _child) {
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
  m_clearanceUtility.SetValidityCheckerLabel(_node.stringXMLParameter("vcLabel2", false, m_vc, "Validity Test Method"));
  m_nf = _node.stringXMLParameter("nfLabel", true, "", "Neighborhood Finder");
  m_dm = _node.stringXMLParameter("dmLabel",true,"","Distance Metric");
  m_lp = _node.stringXMLParameter("lpLabel", true, "", "Local Planning Method");
  m_numDirections = _node.numberXMLParameter("m", false, 1, 1, 1000, "Number of directions to extend");
  m_nc = _node.stringXMLParameter("connectorLabel",false,"","Node Connection Method");
  m_gt = _node.stringXMLParameter("gtype",true,"","Graph type dir/undirected tree/graph");
  if(!_child)
    m_extenderLabel = _node.stringXMLParameter("extenderLabel", true, "", "Extender label");
  m_evaluateGoal = _node.boolXMLParameter("evaluateGoal", false, false, "");
  m_growGoals = _node.boolXMLParameter("growGoals",false,false,"Determines whether or not we grow a tree from the goal");
  m_maxTrial = _node.numberXMLParameter("trial", false, 3, 1, 1000, "Number of trials to get a dispersed direction");

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
BasicRRTStrategy<MPTraits>::Print(ostream& _os) const {
  _os << "BasicRRTStrategy::Print" << endl;
  _os << "\tNeighborhood Finder:: " << m_nf << endl;
  _os << "\tDistance Metric:: " << m_dm << endl;
  _os << "\tValidity Checker:: " << m_vc << endl;
  _os << "\tLocal Planner:: " << m_lp << endl;
  _os << "\tConnection Method:: " << m_nc << endl;
  _os << "\tGraph Type:: " << m_gt << endl;
  _os << "\tExtender:: " << m_extenderLabel << endl;
  _os << "\tEvaluate Goal:: " << m_evaluateGoal << endl;
  _os << "\tEvaluators:: " << endl;
  _os << "\tGrow Goals:: " << m_growGoals << endl;
  typedef vector<string>::const_iterator SIT;
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
  Environment* env = this->GetMPProblem()->GetEnvironment();
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
          && this->GetMPProblem()->GetValidityChecker(m_vc)->IsValid(tmp, callee)){
        VID add = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(tmp);
        m_trees.push_back(vector<VID>(1,add));
      }
      i++;
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
  while(!mapPassedEvaluation) {
    //find my growth direction. Default is too randomly select node or bias towards a goal
    double randomRatio = DRand();
    if(randomRatio < m_growthFocus)
      dir = GoalBiasedDirection();
    else
      dir = this->SelectDirection();

    // Randomize Current Tree
    m_currentTree = m_trees.begin() + LRand()%m_trees.size();
    //cout << "m_trees.size() = " << m_trees.size() << ", currentTree = " << m_currentTree - m_trees.begin() << endl;

    VID recent = this->ExpandTree(dir);
    if(recent != INVALID_VID){

      //connect various trees together
      ConnectTrees(recent);
      //see if tree is connected to goals
      if(m_evaluateGoal)
        EvaluateGoals(recent);

      //evaluate the roadmap
      bool evalMap = this->EvaluateMap(m_evaluators);
      if(!m_growGoals) {
        mapPassedEvaluation = m_trees.size()==1 && evalMap && ((m_evaluateGoal && m_goalsNotFound.size()==0) || !m_evaluateGoal);
        if(this->m_debug && m_goalsNotFound.size()==0)
          cout << "RRT FOUND ALL GOALS" << endl;
      }
      else
        mapPassedEvaluation = evalMap && m_trees.size()==1;
    }
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
  vector<VID> path;
  if(m_query) {
    if(m_evaluateGoal){
      if(m_query->PerformQuery(this->GetMPProblem()->GetRoadmap())){
        if(this->m_debug) cout << "Query successful! Output written to " << str << "." << endl;
      }
      else{
        if(this->m_debug) cout << "Query unsuccessful." << endl;
      }
    }
    path = m_query->GetPathVIDs();
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

  //print roadmap clearance stats
  osStat << endl << endl;
  cout << "Calculating Roadmap Clearance" << endl;
  ClearanceStats cr = m_clearanceUtility.RoadmapClearance();
  cout << "Calculating Path Clearance" << endl;
  ClearanceStats cp = m_clearanceUtility.PathClearance(path);
  osStat
    << "RoadmapAvgClr " << cr.m_avg << endl
    << "RoadmapMinClr " << cr.m_min << endl
    << "RoadmapMaxClr " << cr.m_max << endl
    << "RoadmapVarClr " << cr.m_var << endl
    << "PathAvgClr    " << cp.m_avg << endl
    << "PathMinClr    " << cp.m_min << endl
    << "PathMaxClr    " << cp.m_max << endl
    << "PathVarClr    " << cp.m_var << endl
    << "PathLength    " << cp.m_pathLength << endl;
  osStat.close();

  if(this->m_debug) cout<<"\nEnd Finalizing BasicRRTStrategy"<<endl;
}

template<class MPTraits>
typename MPTraits::CfgType
BasicRRTStrategy<MPTraits>::GoalBiasedDirection(){
  // Determine direction, make sure goal not found
  if (m_goalsNotFound.size() == 0) {
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
typename MPTraits::CfgType
BasicRRTStrategy<MPTraits>::SelectDispersedDirection(VID vd1) {
  StatClass* disperseStatClass = this->GetMPProblem()->GetStatClass();
  string disperseClockName = "disperse sampling time ";
  disperseStatClass->StartClock(disperseClockName);

  CfgType bestCfg;

  typename GraphType::vertex_iterator vi = this->GetMPProblem()->GetRoadmap()->GetGraph()->find_vertex(vd1);
  CfgType c1 = (*vi).property();

  double maxAngle = -MAX_DBL;
  for(size_t i=0; i<m_maxTrial; i++) {
    CfgType randdir = this->SelectDirection();

    //calculating angle between unit vectors
    CfgType difCfg = randdir - c1;
    difCfg = difCfg/difCfg.Magnitude();
    vector<double> v1 = difCfg.GetData();

    //do for all the expanded directions
    vector<CfgType> x = SelectNeighbors(vd1);
    double minAngle = MAX_DBL;
    for(typename vector<CfgType>::iterator vecIT = x.begin(); vecIT!=x.end(); vecIT++) {
      CfgType difCfg2 = *vecIT - c1;
      difCfg2 = difCfg2/difCfg2.Magnitude();
      vector<double> v2 = difCfg2.GetData();
      double res=0;
      for(size_t j=0; j<v1.size(); j++) {
        res+=(v1[j]*v2[j]);
      }

      double angle = acos(res)*180/M_PI;
      if(minAngle > angle)
        minAngle = angle;
    }
    if(maxAngle < minAngle) {
      maxAngle = minAngle;
      bestCfg = randdir;
    }
  }

  disperseStatClass->StopClock(disperseClockName);

  return bestCfg;
}

template<class MPTraits>
vector<typename MPTraits::CfgType>
BasicRRTStrategy<MPTraits>::SelectNeighbors(VID v1){
  typename GraphType::vertex_iterator vi = this->GetMPProblem()->GetRoadmap()->GetGraph()->find_vertex(v1);
  vector<CfgType> vec;
  for(typename GraphType::adj_edge_iterator ei =(*vi).begin(); ei!=(*vi).end(); ei++) {
    VID tgt = (*ei).target();
    CfgType target = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(tgt);
    vec.push_back(target);
  }
  return vec;
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
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dm);
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nf);
  ExtenderPointer e = this->GetMPProblem()->GetExtender(m_extenderLabel);
  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  GraphType* g = rdmp->GetGraph();

  VID recentVID = INVALID_VID;

  // Find closest Cfg in map
  vector<pair<VID, double> > neighbors;
  vector<CfgType> cfgs;

  size_t numRoadmapVertex  = g->get_num_vertices();
  typedef typename vector<vector<VID> >::iterator TRIT;
  size_t treeSize = 0;
  for(TRIT trit = m_trees.begin(); trit!=m_trees.end(); ++trit)
    treeSize += trit->size();

  bool fixTree = false;
  if(treeSize > numRoadmapVertex)
    fixTree = true;
  else {
    vector<pair<size_t, VID> > ccs;
    stapl::sequential::vector_property_map<GraphType, size_t> cmap;
    get_cc_stats(*g, cmap, ccs);
    if(ccs.size() != m_trees.size())
      fixTree = true;
  }

  //node deleted by dynamic environment, fix all trees
  if(fixTree) {
    m_trees.clear();
    vector<pair<size_t, VID> > ccs;
    stapl::sequential::vector_property_map<GraphType, size_t> cmap;
    get_cc_stats(*g, cmap, ccs);
    vector<VID> ccVIDs;
    typename vector<pair<size_t, VID> >::const_iterator ccIt;
    for(ccIt = ccs.begin(); ccIt != ccs.end(); ccIt++) {
      cmap.reset();
      ccVIDs.clear();
      get_cc(*g, cmap, ccIt->second, ccVIDs);
      m_trees.push_back(ccVIDs);
    }
    m_currentTree = m_trees.begin();
  }

  StatClass* stats = this->GetMPProblem()->GetStatClass();
  stats->StartClock("NeighborhoodFinding");
  nf->FindNeighbors(rdmp, m_currentTree->begin(), m_currentTree->end(), _dir, back_inserter(neighbors));
  stats->StopClock("NeighborhoodFinding");

  VID nearVID = neighbors[0].first;
  CfgRef nearest = g->GetVertex(nearVID);

  stats->StartClock("Extend");
  vector<CfgType> intermediates, rintermediates;
  CfgType newCfg;
  bool extendSucc = e->Extend(nearest, _dir, newCfg, intermediates);
  double dist;
  if(intermediates.empty())
    dist = dm->Distance(nearest, newCfg);
  else {
    dist = dm->Distance(nearest, intermediates.front());
    typedef typename vector<CfgType>::iterator CIT;
    for(CIT cit1 = intermediates.begin(), cit2 = cit1 + 1; cit2 != intermediates.end(); ++cit1, ++cit2)
      dist += dm->Distance(*cit1, *cit2);
    dist += dm->Distance(intermediates.back(), newCfg);
    copy(intermediates.rbegin(), intermediates.rend(), back_inserter(rintermediates));
  }
  stats->StopClock("Extend");

  //failed expansion return
  if(!extendSucc) {
    if(this->m_debug) cout << "RRT could not expand!" << endl;
    return recentVID;
  }

  static size_t expansions = 0;
  cout << "Expansion:: " << ++expansions << "\tto " << newCfg << endl;

  // If good to go, add to roadmap
  if(dist >= m_minDist ) {
    recentVID = g->AddVertex(newCfg);
    g->GetVertex(recentVID).SetStat("Parent", neighbors[0].first);
    m_currentTree->push_back(recentVID);

    if(m_gt.find("UNDIRECTED") != string::npos)
      g->AddEdge(nearVID, recentVID,
          make_pair(WeightType("RRTExpand", dist, intermediates),
            WeightType("RRTExpand", dist, rintermediates)));
    else
      g->AddEdge(nearVID, recentVID, WeightType("RRTExpand", dist, intermediates));

    if(m_gt.find("GRAPH") != string::npos)
      ConnectNeighbors(recentVID, neighbors[0].first);

    //expansion to other m-1 directions
    for(size_t i = 2; i <= m_numDirections; ++i) {
      CfgType randdir = this->SelectDispersedDirection(nearVID);

      stats->StartClock("Extend");
      vector<CfgType> intermediates, rintermediates;
      bool extendSucc = e->Extend(nearest, randdir, newCfg, intermediates);
      if(intermediates.empty())
        dist = dm->Distance(nearest, newCfg);
      else {
        dist = dm->Distance(nearest, intermediates.front());
        typedef typename vector<CfgType>::iterator CIT;
        for(CIT cit1 = intermediates.begin(), cit2 = cit1 + 1;
            cit2 != intermediates.end(); ++cit1, ++cit2)
          dist += dm->Distance(*cit1, *cit2);
        dist += dm->Distance(intermediates.back(), newCfg);
        copy(intermediates.rbegin(), intermediates.rend(), back_inserter(rintermediates));
      }
      stats->StopClock("Extend");

      if(!extendSucc) {
        if(this->m_debug) cout << "RRT could not expand to additional directions!" << endl;
      }
      else if(dist >= m_minDist) {
        VID otherVID = g->AddVertex(newCfg);
        g->GetVertex(otherVID).SetStat("Parent", nearVID);
        m_currentTree->push_back(otherVID);

        if(m_gt.find("UNDIRECTED") != string::npos)
          g->AddEdge(nearVID, otherVID,
              make_pair(WeightType("RRTExpand", dist, intermediates),
                WeightType("RRTExpand", dist, rintermediates)));
        else
          g->AddEdge(nearVID, otherVID, WeightType("RRTExpand", dist, intermediates));

        if(m_gt.find("GRAPH") != string::npos) {
          stats->StartClock("Connection");
          ConnectNeighbors(otherVID, nearVID);
          stats->StopClock("Connection");
        }
      }
    }
  }

  return recentVID;
}

template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::ConnectTrees(VID _recentlyGrown) {
  //Setup MP variables
  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dm);
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nf);
  CDInfo  cdInfo;

  if(m_trees.size()==1) return; //trees connected

  CfgType c1 = rdmp->GetGraph()->GetVertex(_recentlyGrown);

  VID closestNode = INVALID_VID;
  CfgType closestCfg;
  double minDis = MAX_DBL;
  typename vector<vector<VID> >::iterator treeClosest = m_currentTree;
  CfgType c2;
  typedef typename vector<vector<VID> >::iterator TRIT;

  for(TRIT trit = m_trees.begin(); trit!=m_trees.end(); ++trit){
    if(trit != m_currentTree){
      vector<pair<VID, double> > closest;
      nf->FindNeighbors(rdmp, trit->begin(), trit->end(), c1, back_inserter(closest));
      if(closest.size() != 0) {
        c2  = rdmp->GetGraph()->GetVertex(closest[0].first);
        double dist = dm->Distance(c1,c2);
        if(dist<minDis){
          treeClosest = trit;
          minDis = dist;
          closestNode = closest[0].first;
          closestCfg = c2;
        }
      }
    }
  }

  CfgType newCfg;
  int weight = 0;
  vector<CfgType> intermediateNodes;
  if(!this->GetMPProblem()->GetExtender(m_extenderLabel)
      ->Extend(closestCfg, c1, newCfg, intermediateNodes)) {
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
    else if(dm->Distance(newCfg, c2) >= m_minDist ) {
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
BasicRRTStrategy<MPTraits>::EvaluateGoals(VID _newVID){
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dmp = this->GetMPProblem()->GetDistanceMetric(m_dm);
  LocalPlannerPointer lpp = this->GetMPProblem()->GetLocalPlanner(m_lp);
  GraphType* rdmp = this->GetMPProblem()->GetRoadmap()->GetGraph();

  CfgType& qnew = rdmp->GetVertex(_newVID);
  LPOutput<MPTraits> lpOutput;
  // Check if goals have been found
  vector<size_t>::iterator i = m_goalsNotFound.begin();
  while(i != m_goalsNotFound.end()) {
    double dist = dmp->Distance(m_goals[*i], qnew);
    if(this->m_debug) cout << "Distance to goal::" << dist << endl;
    CfgType col;
    if(dist < m_delta && lpp->IsConnected(qnew, m_goals[*i], col, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true, false, false)){
      if(this->m_debug) cout << "Goal found::" << m_goals[*i] << endl;
      VID goalVID;
      if(!(rdmp->IsVertex( m_goals[*i])))
        goalVID = rdmp->AddVertex(m_goals[*i]);
      else
        goalVID = rdmp->GetVID(m_goals[*i]);
      rdmp->AddEdge(_newVID, goalVID, lpOutput.m_edge);
      i = m_goalsNotFound.erase(i);
    }
    else
      ++i;
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
