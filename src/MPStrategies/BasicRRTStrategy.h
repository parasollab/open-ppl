#ifndef BASIC_RRT_STRATEGY_H_
#define BASIC_RRT_STRATEGY_H_

#include "MPStrategyMethod.h"

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
        bool _evaluateGoal=true, const CfgType& _start=CfgType(),
        const CfgType& _goal=CfgType(), size_t _numRoots=1,
        size_t _numDirections=1, size_t _maxTrial = 3, bool _growGoals=false);

    BasicRRTStrategy(MPProblemType* _problem, XMLNode& _node,
        bool _child = false);

    virtual ~BasicRRTStrategy();

    virtual void ParseXML(XMLNode& _node, bool _child = false);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

    void SetMPProblem(MPProblemType* _problem);

    //return path computed if RRT is successfully run with a query
    vector<CfgType> GetPath() {return m_query->GetPath();}

  protected:
    // Helper functions
    CfgType GoalBiasedDirection();
    CfgType SelectDirection();
    CfgType SelectDispersedDirection(VID _v);
    vector<CfgType> SelectNeighbors(VID _v);
    virtual VID ExpandTree(CfgType& _dir);
    void ConnectTrees(VID _recentlyGrown);
    void ConnectNeighbors(VID _newVID);
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
    size_t m_numRoots, m_numDirections, m_maxTrial;
    bool m_growGoals;
    vector< vector<VID> > m_trees;

    typename vector<vector<VID> >::iterator m_currentTree;
    vector<CfgType> m_goals;
    vector<size_t> m_goalsNotFound;
};

template<class MPTraits>
BasicRRTStrategy<MPTraits>::
BasicRRTStrategy(string _lp, string _dm, string _nf, string _vc, string _nc,
    string _gt, string _extenderLabel, vector<string> _evaluators,
    double _delta, double _minDist, double _growthFocus, bool _evaluateGoal,
    const CfgType& _start, const CfgType& _goal, size_t _numRoots,
    size_t _numDirections, size_t _maxTrial, bool _growGoals) :
  m_evaluators(_evaluators), m_lp(_lp), m_dm(_dm), m_nf(_nf), m_vc(_vc),
  m_query(new Query<MPTraits>(_start, _goal)), m_nc(_nc), m_gt(_gt),
  m_extenderLabel(_extenderLabel), m_delta(_delta), m_minDist(_minDist),
  m_growthFocus(_growthFocus), m_evaluateGoal(_evaluateGoal),
  m_numRoots(_numRoots), m_numDirections(_numDirections), m_maxTrial(_maxTrial),
  m_growGoals(_growGoals) {
    this->SetName("BasicRRTStrategy");
  }

template<class MPTraits>
BasicRRTStrategy<MPTraits>::
BasicRRTStrategy(MPProblemType* _problem, XMLNode& _node, bool _child) :
  MPStrategyMethod<MPTraits>(_problem, _node),
  m_query((Query<MPTraits>*)NULL) {
    this->SetName("BasicRRTStrategy");
    ParseXML(_node, _child);
  }

template<class MPTraits>
BasicRRTStrategy<MPTraits>::
~BasicRRTStrategy() {
}

template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::
ParseXML(XMLNode& _node, bool _child) {
  for(auto& child : _node)
    if(child.Name() == "Evaluator")
      m_evaluators.push_back(
          child.Read("label", true, "", "Evaluation Method"));

  m_delta = _node.Read("delta", false, 1.0, 0.0, MAX_DBL,
      "Delta Distance");
  m_minDist = _node.Read("minDist", false, 0.0, 0.0, m_delta,
      "Minimum Distance");
  m_numRoots = _node.Read("numRoots", false, 1, 0, MAX_INT,
      "Number of Roots");
  m_growthFocus = _node.Read("growthFocus", false, 0.0, 0.0, 1.0,
      "#GeneratedTowardsGoal/#Generated");
  m_vc = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_nf = _node.Read("nfLabel", true, "", "Neighborhood Finder");
  m_dm = _node.Read("dmLabel",true,"","Distance Metric");
  m_lp = _node.Read("lpLabel", true, "", "Local Planning Method");
  m_numDirections = _node.Read("m", false, 1, 1, 1000,
      "Number of directions to extend");
  m_nc = _node.Read("connectorLabel", false, "",
      "Node Connection Method");
  m_gt = _node.Read("gtype", true, "",
      "Graph type dir/undirected tree/graph");
  if(!_child)
    m_extenderLabel = _node.Read("extenderLabel", true, "",
        "Extender label");
  m_evaluateGoal = _node.Read("evaluateGoal", false, false, "");
  m_growGoals = _node.Read("growGoals", false, false,
      "Determines whether or not we grow a tree from the goal");
  m_maxTrial = _node.Read("trial", false, 3, 1, 1000,
      "Number of trials to get a dispersed direction");

  //optionally read in a query and create a Query object.
  string query = _node.Read("query", false, "", "Query Filename");
  if(query != "") {
    m_query = shared_ptr<Query<MPTraits>>(new Query<MPTraits>(query));
    m_query->SetMPProblem(this->GetMPProblem());
    m_query->SetDebug(this->m_debug);
  }
}

template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::
Print(ostream& _os) const {
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
  for(auto&  s : m_evaluators)
    _os << "\t\t" << s << endl;
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
BasicRRTStrategy<MPTraits>::
Initialize(){
  if(this->m_debug)
    cout<<"\nInitializing BasicRRTStrategy::"<<endl;

  // Setup MP variables
  Environment* env = this->GetEnvironment();
  string callee = "BasicRRTStrategy::RRT";
  // Setup RRT Variables
  CfgType tmp;
  if(m_query) {
    vector<CfgType>& queryCfgs = m_query->GetQuery();
    typedef typename vector<CfgType>::iterator CIT;
    if(m_growGoals) {
      for(auto&  cfg : queryCfgs) {
        VID add = this->GetRoadmap()->GetGraph()->AddVertex(cfg);
        m_trees.push_back(vector<VID>(1, add));
      }
    }
    else {
      for(CIT cit1 = queryCfgs.begin(), cit2 = cit1+1; cit2!=queryCfgs.end();
          cit1++, cit2++) {
        VID add = this->GetRoadmap()->GetGraph()->AddVertex(*cit1);
        m_trees.push_back(vector<VID>(1, add));
        m_goals.push_back(*cit2);
        m_goalsNotFound.push_back(m_goals.size()-1);
      }
    }
  }
  else{
    // Add root vertex/vertices
    size_t i = 0;
    while(i < m_numRoots){
      //tmp.GetRandomCfg(env);
      if(env->InBounds(tmp)
          && this->GetValidityChecker(m_vc)->IsValid(tmp, callee)){
        VID add = this->GetRoadmap()->GetGraph()->AddVertex(tmp);
        m_trees.push_back(vector<VID>(1,add));
      }
      i++;
    }
  }

  if(this->m_debug) {
    cout << "there are " << m_trees.size() << " trees\n";
    if(!m_trees.empty()) {
      cout << "\tthey are:\n";
      size_t t = 0;
      for(auto&  tree : m_trees) {
        cout << "tree " << t++ << " has " << tree.size() << " vertices";
        if(!tree.empty()) {
          cout << " with root " << *tree.begin() << ":\n";
          cout << this->GetRoadmap()->GetGraph()->GetVertex(*tree.begin());
        }
        cout << endl;
      }
    }
    cout << "there are " << m_goals.size() << " goals\n";
    if(!m_goals.empty()) {
      cout << "\tthey are:\n\t";
      copy(m_goals.begin(), m_goals.end(), ostream_iterator<CfgType>(cout, "\n\t"));
      cout << endl;
    }
    cout << "there are " << m_goalsNotFound.size() << " goalsNotFound\n";
  }

  m_currentTree = m_trees.begin();//set initial tree to be grown

  if(this->m_debug)
    cout<<"\nEnding Initializing BasicRRTStrategy"<<endl;
}

////////////////
//Run/Start Phase
////////////////
template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::
Run() {
  if(this->m_debug)
    cout << "\nRunning BasicRRTStrategy::" << endl;

  // Setup MP Variables
  StatClass* stats = this->GetStatClass();

  stats->StartClock("RRT Generation");

  CfgType dir;
  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation) {
    //find my growth direction. Default is too randomly select node or bias
    //towards a goal
    double randomRatio = DRand();
    if(randomRatio < m_growthFocus) {
      dir = GoalBiasedDirection();
      if(this->m_debug)
        cout << "goal biased direction selected: " << dir << endl;
    }
    else {
      dir = this->SelectDirection();
      if(this->m_debug)
        cout << "random direction selected: " << dir << endl;
    }

    // Randomize Current Tree
    m_currentTree = m_trees.begin() + LRand()%m_trees.size();
    if(this->m_debug)
      cout << "m_trees.size() = " << m_trees.size()
        << ", currentTree = " << distance(m_trees.begin(), m_currentTree) << endl;

    VID recent = this->ExpandTree(dir);
    if(recent != INVALID_VID) {

      //connect various trees together
      ConnectTrees(recent);
      //see if tree is connected to goals
      if(m_evaluateGoal)
        EvaluateGoals(recent);

      //evaluate the roadmap
      bool evalMap = this->EvaluateMap(m_evaluators);
      if(!m_growGoals) {
        mapPassedEvaluation = m_trees.size()==1 && evalMap &&
          ((m_evaluateGoal && m_goalsNotFound.size()==0) || !m_evaluateGoal);
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
BasicRRTStrategy<MPTraits>::
Finalize() {
  RoadmapType* rdmp = this->GetRoadmap();
  string base = this->GetBaseFilename();

  //perform query if query was given as input
  vector<VID> path;
  if(m_query) {
    if(m_evaluateGoal) {
      if(m_query->PerformQuery(rdmp) && this->m_debug)
        cout << "Query successful!" << endl;
      else if(this->m_debug)
        cout << "Query unsuccessful." << endl;
    }
    path = m_query->GetPathVIDs();
  }

  //output final map
  rdmp->Write(base + ".map", this->GetEnvironment());

  //output stats
  string str = base + ".stat";
  ofstream  osStat(str.c_str());
  StatClass* stats = this->GetStatClass();
  stats->PrintAllStats(osStat, rdmp);
}

template<class MPTraits>
typename MPTraits::CfgType
BasicRRTStrategy<MPTraits>::
GoalBiasedDirection() {
  // Determine direction, make sure goal not found
  if(m_goalsNotFound.empty())
    return CfgType();
  else {
    size_t goalNum = LRand()%m_goalsNotFound.size();
    return m_goals[m_goalsNotFound[goalNum]];
  }
}

template<class MPTraits>
typename MPTraits::CfgType
BasicRRTStrategy<MPTraits>::
SelectDirection(){
  CfgType dir;
  dir.GetRandomCfg(this->GetEnvironment());
  return dir;
}

template<class MPTraits>
typename MPTraits::CfgType
BasicRRTStrategy<MPTraits>::
SelectDispersedDirection(VID _v) {
  StatClass* stats = this->GetStatClass();

  string disperseClock = "disperse sampling time";
  stats->StartClock(disperseClock);

  CfgType bestCfg;
  CfgType c1 = this->GetRoadmap()->GetGraph()->GetVertex(_v);
  vector<CfgType> x = SelectNeighbors(_v);

  double maxAngle = -MAX_DBL;
  for(size_t i = 0; i < m_maxTrial; i++) {
    CfgType randdir = this->SelectDirection();

    //calculating angle between unit vectors
    CfgType difCfg = randdir - c1;
    difCfg = difCfg/difCfg.Magnitude();

    //do for all the expanded directions
    double minAngle = MAX_DBL;
    for(auto&  cfg : x) {
      CfgType difCfg2 = cfg - c1;
      difCfg2 = difCfg2/difCfg2.Magnitude();
      double res=0;
      for(size_t j = 0; j < difCfg.DOF(); ++j) {
        res+=(difCfg[j]*difCfg2[j]);
      }

      double angle = acos(res);
      if(minAngle > angle)
        minAngle = angle;
    }
    if(maxAngle < minAngle) {
      maxAngle = minAngle;
      bestCfg = randdir;
    }
  }

  stats->StopClock(disperseClock);

  return bestCfg;
}

template<class MPTraits>
vector<typename MPTraits::CfgType>
BasicRRTStrategy<MPTraits>::
SelectNeighbors(VID _v) {
  GraphType* g = this->GetRoadmap()->GetGraph();
  typename GraphType::vertex_iterator vi = g->find_vertex(_v);
  vector<CfgType> vec;
  for(const auto&  e : *vi)
    vec.push_back(g->GetVertex(e.target()));
  return vec;
}

template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::
ConnectNeighbors(VID _newVID) {
  if(_newVID != INVALID_VID) {
    vector<VID> currentVID(1, _newVID);

    ConnectorPointer c = this->GetConnector(m_nc);

    StatClass* stats = this->GetStatClass();
    string conClock = "Total Connection time ";
    stats->StartClock(conClock);

    c->Connect(this->GetRoadmap(),
        currentVID.begin(), currentVID.end(),
        m_currentTree->begin(), m_currentTree->end());

    stats->StopClock(conClock);
  }
}

template<class MPTraits>
typename BasicRRTStrategy<MPTraits>::VID
BasicRRTStrategy<MPTraits>::
ExpandTree(CfgType& _dir) {
  // Setup MP Variables
  StatClass* stats = this->GetStatClass();
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dm);
  NeighborhoodFinderPointer nf = this->GetNeighborhoodFinder(m_nf);
  ExtenderPointer e = this->GetExtender(m_extenderLabel);
  LocalPlannerPointer lp = this->GetLocalPlanner(m_lp);
  RoadmapType* rdmp = this->GetRoadmap();
  GraphType* g = rdmp->GetGraph();

  VID recentVID = INVALID_VID;

  // Find closest Cfg in map
  vector<pair<VID, double> > neighbors;
  vector<CfgType> cfgs;

  size_t numRoadmapVertex  = g->get_num_vertices();

  size_t treeSize = 0;
  for(auto&  tree : m_trees)
    treeSize += tree.size();

  bool fixTree = false;
  if(treeSize > numRoadmapVertex)
    fixTree = true;
  else {
    vector<pair<size_t, VID>> ccs;
    stapl::sequential::vector_property_map<GraphType, size_t> cmap;
    get_cc_stats(*g, cmap, ccs);
    if(ccs.size() != m_trees.size())
      fixTree = true;
  }

  //node deleted by dynamic environment, fix all trees
  if(fixTree) {
    m_trees.clear();
    vector<pair<size_t, VID>> ccs;
    stapl::sequential::vector_property_map<GraphType, size_t> cmap;
    get_cc_stats(*g, cmap, ccs);
    vector<VID> ccVIDs;
    for(auto&  cc : ccs) {
      cmap.reset();
      ccVIDs.clear();
      get_cc(*g, cmap, cc.second, ccVIDs);
      m_trees.push_back(ccVIDs);
    }
    m_currentTree = m_trees.begin();
  }

  stats->StartClock("NeighborhoodFinding");

  nf->FindNeighbors(this->GetRoadmap(), m_currentTree->begin(),
      m_currentTree->end(), _dir, back_inserter(neighbors));

  stats->StopClock("NeighborhoodFinding");

  VID nearVID = neighbors[0].first;
  CfgRef nearest = g->GetVertex(neighbors[0].first);
  CfgType newCfg;
  vector<CfgType> intermediates, rintermediates;

  stats->StartClock("Extend");
  LPOutput<MPTraits> lpOutput;
  bool extendSucc = e->Extend(nearest, _dir, newCfg, lpOutput);
  intermediates = lpOutput.m_intermediates;
  stats->StopClock("Extend");

  double dist;
  if(intermediates.empty())
    dist = dm->Distance(nearest, newCfg);
  else {
    dist = dm->Distance(nearest, intermediates.front());
    typedef typename vector<CfgType>::iterator CIT;
    for(CIT cit1 = intermediates.begin(),
        cit2 = cit1 + 1; cit2 != intermediates.end(); ++cit1, ++cit2)
      dist += dm->Distance(*cit1, *cit2);
    dist += dm->Distance(intermediates.back(), newCfg);
    copy(intermediates.rbegin(), intermediates.rend(),
        back_inserter(rintermediates));
  }

  //failed expansion return
  if(!extendSucc) {
    if(this->m_debug) cout << "RRT could not expand!" << endl;
    return recentVID;
  }

  static size_t expansions = 0;
  cout << "Expansion:: " << ++expansions << "\tto " << newCfg << endl;

  if(this->m_debug)
    cout << "RRT from " << nearVID
      << " expanded to " << newCfg << endl;

  // If good to go, add to roadmap
  if(dist >= m_minDist ) {
    recentVID = g->AddVertex(newCfg);
    g->GetVertex(recentVID).SetStat("Parent", neighbors[0].first);
    m_currentTree->push_back(recentVID);
    if(std::string::npos != m_gt.find("UNDIRECTED"))
      g->AddEdge(nearVID, recentVID, lpOutput.m_edge);
    else
      g->AddEdge(nearVID, recentVID, lpOutput.m_edge.first);

    //If Graph type is GRAPH not TREE, then perform connection phase, i.e., this
    //is what RRG goes.
    if(std::string::npos != m_gt.find("GRAPH")) {
      if(this->m_debug) {
        cout << "tree roots:\n";
        for(auto&  tree : m_trees)
          cout << "\t" << tree.front() << " (" << tree.size() << ")\n";
        cout << "connecting neighbors...\n";
      }
      ConnectNeighbors(recentVID);
      if(this->m_debug) {
        cout << "tree roots:\n";
        for(auto&  tree : m_trees)
          cout << "\t" << tree.front() << " (" << tree.size() << ")\n";
      }
    }

    for(size_t i=2; i<=m_numDirections; i++) {
      //expansion to other m-1 directions
      CfgType randdir = this->SelectDispersedDirection(nearVID);
      stats->StartClock("Extend");
      vector<CfgType> intermediates, rintermediates;
      bool extendSucc = e->Extend(nearest, randdir, newCfg, lpOutput);
      intermediates = lpOutput.m_intermediates;
      if(intermediates.empty())
        dist = dm->Distance(nearest, newCfg);
      else {
        dist = dm->Distance(nearest, intermediates.front());
        typedef typename vector<CfgType>::iterator CIT;
        for(CIT cit1 = intermediates.begin(), cit2 = cit1 + 1;
            cit2 != intermediates.end(); ++cit1, ++cit2)
          dist += dm->Distance(*cit1, *cit2);
        dist += dm->Distance(intermediates.back(), newCfg);
        copy(intermediates.rbegin(), intermediates.rend(),
            back_inserter(rintermediates));
      }
      stats->StopClock("Extend");

      if(!extendSucc) {
        if(this->m_debug)
          cout << "RRT could not expand to additional directions!" << endl;
      }
      else if(dist >= m_minDist) {
        VID otherVID = g->AddVertex(newCfg);
        g->GetVertex(otherVID).SetStat("Parent", nearVID);
        m_currentTree->push_back(otherVID);
        if(std::string::npos!=m_gt.find("UNDIRECTED"))
          g->AddEdge(nearVID, otherVID, lpOutput.m_edge);
        else
          g->AddEdge(nearVID, otherVID, lpOutput.m_edge.first);

        if(std::string::npos!=m_gt.find("GRAPH")){
          if(this->m_debug) {
            cout << "tree roots:\n";
            for(auto&  tree : m_trees)
              cout << "\t" << tree.front() << " (" << tree.size() << ")\n";
          }
          string conClockName = "Connection time ";
          stats->StartClock(conClockName);

          ConnectNeighbors(otherVID);

          stats->StopClock(conClockName);
          if(this->m_debug) {
            cout << "tree roots:\n";
            for(auto&  tree : m_trees)
              cout << "\t" << tree.front() << " (" << tree.size() << ")\n";
          }
        }
      }
    }
  }
  else {
    if(this->m_debug)
      cout << "\t(expansion too close, not adding)\n";
  }

  return recentVID;
}

template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::
ConnectTrees(VID _recentlyGrown) {
  //Setup MP variables
  RoadmapType* rdmp = this->GetRoadmap();
  GraphType* g = rdmp->GetGraph();
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dm);
  NeighborhoodFinderPointer nf = this->GetNeighborhoodFinder(m_nf);
  CDInfo  cdInfo;

  //trees are connected
  if(m_trees.size() == 1)
    return;

  CfgType c1 = g->GetVertex(_recentlyGrown);

  VID closestNode = INVALID_VID;
  CfgType closestCfg;
  double minDis = MAX_DBL;
  typename vector<vector<VID> >::iterator treeClosest = m_currentTree;
  CfgType c2;
  typedef typename vector<vector<VID> >::iterator TRIT;

  for(TRIT trit = m_trees.begin(); trit!=m_trees.end(); ++trit) {
    if(trit != m_currentTree) {
      vector<pair<VID, double>> closest;
      nf->FindNeighbors(rdmp, trit->begin(), trit->end(), c1,
          back_inserter(closest));
      if(closest.size() != 0) {
        c2  = g->GetVertex(closest[0].first);

        double dist = dm->Distance(c1, c2);
        if(dist < minDis) {
          treeClosest = trit;
          minDis = dist;
          closestNode = closest[0].first;
          closestCfg = c2;
        }
      }
    }
  }
  if(this->m_debug) {
    cout << "recently grown = " << _recentlyGrown << "\tclosest = "
      << closestNode << "\tdist = " << minDis << endl;
  }

  CfgType newCfg;
  LPOutput<MPTraits> lpOutput;
  if(!this->GetExtender(m_extenderLabel)
      ->Extend(closestCfg, c1, newCfg, lpOutput)) {
    if(this->m_debug)
      cout << "RRT could not expand in connection phase, trapped!" << endl;
    return;
  }
  //add node to roadmap provided it's faraway enough or connects two trees
  else{
    VID newVID;
    //we connect the trees
    if(c1 == newCfg) {
      if(distance(m_trees.begin(), m_currentTree) >
          distance(m_trees.begin(), treeClosest)) {
        swap(m_currentTree, treeClosest);
      }
      m_currentTree->insert(m_currentTree->end(), treeClosest->begin(),
          treeClosest->end());
      m_trees.erase(treeClosest);
      newVID = _recentlyGrown;
      if(this->m_debug)
        cout << "We connected the trees (expansion length = "
          << dm->Distance(c1, c2) << ")" << endl;
    }
    else if(dm->Distance(newCfg, c2) >= m_minDist) {
      newVID = g->AddVertex(newCfg);
      treeClosest->push_back(newVID);
      m_currentTree = treeClosest;
      if(this->m_debug)
        cout << "connecting trees: did not expand all the way but far enough to"
          << "add.  added vid " << newVID << "\tdist = "
          << dm->Distance(newCfg, c2) << endl;
    }
    //too close to add, keep expanding current tree
    else {
      if(this->m_debug)
        cout << "connecting trees: did not expand all the way and too close to "
          << "add.\n";
      return;
    }

    //add to roadmap
    if(std::string::npos != m_gt.find("UNDIRECTED")){
      g->AddEdge(closestNode, newVID, lpOutput.m_edge);
    }
    else
      g->AddEdge(closestNode, newVID, lpOutput.m_edge.first);
    g->GetVertex(closestNode).SetStat("Parent", newVID);

    if(std::string::npos != m_gt.find("GRAPH")) {
      if(this->m_debug) {
        cout << "tree roots:\n";
        for(auto&  tree : m_trees)
          cout << "\t" << tree.front() << " (" << tree.size() << ")\n";
      }
      ConnectNeighbors(newVID);
      if(this->m_debug) {
        cout << "tree roots:\n";
        for(auto&  tree : m_trees)
          cout << "\t" << tree.front() << " (" << tree.size() << ")\n";
      }
    }
  }
}

template<class MPTraits>
void
BasicRRTStrategy<MPTraits>::
EvaluateGoals(VID _newVID) {
  // Setup MP Variables
  Environment* env = this->GetEnvironment();
  DistanceMetricPointer dmp = this->GetDistanceMetric(m_dm);
  LocalPlannerPointer lpp = this->GetLocalPlanner(m_lp);
  GraphType* rdmp = this->GetRoadmap()->GetGraph();

  CfgType& qnew = rdmp->GetVertex(_newVID);
  LPOutput<MPTraits> lpOutput;
  // Check if goals have been found
  vector<size_t>::iterator i = m_goalsNotFound.begin();
  while(i != m_goalsNotFound.end()) {
    double dist = dmp->Distance(m_goals[*i], qnew);
    if(this->m_debug)
      cout << "Distance to goal::" << dist << endl;
    CfgType col;
    if(dist < m_delta && lpp->IsConnected(qnew, m_goals[*i], col, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true, false, false)){
      if(this->m_debug)
        cout << "Goal found::" << m_goals[*i] << endl;
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
BasicRRTStrategy<MPTraits>::
SetMPProblem(MPProblemType* _problem) {
  MPBaseObject<MPTraits>::SetMPProblem(_problem);
  if(m_query)
    m_query->SetMPProblem(_problem);
}

#endif
