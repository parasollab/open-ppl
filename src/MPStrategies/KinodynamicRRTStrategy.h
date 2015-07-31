#ifndef KINODYNAMIC_RRT_STRATEGY_H_
#define KINODYNAMIC_RRT_STRATEGY_H_

#include "MPStrategyMethod.h"

#include "MapEvaluators/Query.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class KinodynamicRRTStrategy : public MPStrategyMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType StateType;
    typedef typename MPTraits::CfgRef StateRef;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::ExtenderPointer ExtenderPointer;

    //Non-XML constructor sets all private variables
    KinodynamicRRTStrategy(string _dm = "", string _nf = "", string _vc= "",
        string _extenderLabel = "",
        vector<string> _evaluators = vector<string>(),
        double _goalDist = 10.0, double _minDist = 0.01,
        double _growthFocus = 0.05, bool _evaluateGoal = true,
        const StateType& _start=StateType(), const StateType& _goal=StateType());

    KinodynamicRRTStrategy(MPProblemType* _problem, XMLNode& _node);

    virtual ~KinodynamicRRTStrategy();

    virtual void ParseXML(XMLNode& _node);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();
    virtual void Print(ostream& _os) const;

    void SetMPProblem(MPProblemType* _problem);

    //return path computed if RRT is successfully run with a query
    vector<StateType> GetPath() {return m_query->GetPath();}

  protected:
    // Helper functions
    StateType GoalBiasedDirection();
    StateType SelectDirection();
    virtual VID ExpandTree(StateType& _dir);
    void EvaluateGoals(VID _newVID);

    string m_dm;
    string m_nf;
    string m_vc;
    shared_ptr<Query<MPTraits> > m_query;
    string m_extenderLabel;
    double m_goalDist, m_minDist, m_growthFocus;
    bool m_evaluateGoal;

    vector<StateType> m_goals;
    vector<size_t> m_goalsNotFound;
};

template<class MPTraits>
KinodynamicRRTStrategy<MPTraits>::
KinodynamicRRTStrategy(string _dm, string _nf, string _vc,
    string _extenderLabel, vector<string> _evaluators,
    double _goalDist, double _minDist, double _growthFocus, bool _evaluateGoal,
    const StateType& _start, const StateType& _goal) :
  m_dm(_dm), m_nf(_nf), m_vc(_vc),
  m_query(new Query<MPTraits>(_start, _goal)),
  m_extenderLabel(_extenderLabel), m_goalDist(_goalDist), m_minDist(_minDist),
  m_growthFocus(_growthFocus), m_evaluateGoal(_evaluateGoal) {
    this->SetName("KinodynamicRRTStrategy");
    this->m_meLabels = _evaluators;
  }

template<class MPTraits>
KinodynamicRRTStrategy<MPTraits>::
KinodynamicRRTStrategy(MPProblemType* _problem, XMLNode& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node),
  m_query( (Query<MPTraits>*)NULL ) {
    this->SetName("KinodynamicRRTStrategy");
    ParseXML(_node);
  }

template<class MPTraits>
KinodynamicRRTStrategy<MPTraits>::
~KinodynamicRRTStrategy() {
}

template<class MPTraits>
void
KinodynamicRRTStrategy<MPTraits>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node)
    if(child.Name() == "Evaluator")
      this->m_meLabels.push_back(
          child.Read("label", true, "", "Evaluation Method"));

  m_minDist = _node.Read("minDist", false, 0.0, 0.0, MAX_DBL,
      "Minimum Distance");
  m_growthFocus = _node.Read("growthFocus", false, 0.0, 0.0, 1.0,
      "#GeneratedTowardsGoal/#Generated");
  m_vc = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_nf = _node.Read("nfLabel", true, "", "Neighborhood Finder");
  m_dm = _node.Read("dmLabel",true,"","Distance Metric");
  m_extenderLabel = _node.Read("extenderLabel", true, "",
      "Extender label");
  //optionally read in a query and create a Query object.
  string query = _node.Read("query", false, "", "Query Filename");
  m_evaluateGoal = _node.Read("evaluateGoal", !query.empty(), false, "");
  m_goalDist = _node.Read("goalDist", m_evaluateGoal, 1.0, 0.0, MAX_DBL,
      "Delta Distance");

  if(query != "") {
    m_query = shared_ptr<Query<MPTraits>>(new Query<MPTraits>(query));
    m_query->SetMPProblem(this->GetMPProblem());
    m_query->SetDebug(this->m_debug);
  }
}

template<class MPTraits>
void
KinodynamicRRTStrategy<MPTraits>::
Print(ostream& _os) const {
  _os << "KinodynamicRRTStrategy::Print" << endl;
  _os << "\tNeighborhood Finder:: " << m_nf << endl;
  _os << "\tDistance Metric:: " << m_dm << endl;
  _os << "\tValidity Checker:: " << m_vc << endl;
  _os << "\tExtender:: " << m_extenderLabel << endl;
  _os << "\tEvaluate Goal:: " << m_evaluateGoal << endl;
  _os << "\tEvaluators:: " << endl;
  for(auto&  s : this->m_meLabels)
    _os << "\t\t" << s << endl;
  _os << "\tgoalDist:: " << m_goalDist << endl;
  _os << "\tminimum distance:: " << m_minDist << endl;
  _os << "\tgrowth focus:: " << m_growthFocus << endl;
}

template<class MPTraits>
void
KinodynamicRRTStrategy<MPTraits>::
Initialize() {
  if(m_query) {
    vector<StateType>& queryCfgs = m_query->GetQuery();
    typedef typename vector<StateType>::iterator CIT;
    for(CIT cit1 = queryCfgs.begin(), cit2 = cit1+1; cit2!=queryCfgs.end();
        cit1++, cit2++) {
      VID add = this->GetRoadmap()->GetGraph()->AddVertex(*cit1);
      m_goals.push_back(*cit2);
      m_goalsNotFound.push_back(m_goals.size()-1);
    }
  }
  else{
    // Add root vertex/vertices
    StateType tmp;
    Environment* env = this->GetEnvironment();
    string callee = "KinodynamicRRTStrategy::RRT";
    while(true){
      tmp.GetRandomCfg(env);
      if(env->InBounds(tmp)
          && this->GetValidityChecker(m_vc)->IsValid(tmp, callee)){
        VID add = this->GetRoadmap()->GetGraph()->AddVertex(tmp);
        break;
      }
    }
  }
}

template<class MPTraits>
void
KinodynamicRRTStrategy<MPTraits>::
Run() {
  if(this->m_debug)
    cout << "\nRunning KinodynamicRRTStrategy::" << endl;

  // Setup MP Variables
  StatClass* stats = this->GetStatClass();

  stats->StartClock("RRT Generation");

  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation) {
    //find my growth direction. Default is too randomly select node or bias
    //towards a goal
    StateType dir;
    double randomRatio = DRand();
    if(randomRatio < m_growthFocus) {
      dir = GoalBiasedDirection();
      if(this->m_debug)
        cout << "goal biased direction selected: " << dir << endl;
    }
    else {
      dir = SelectDirection();
      if(this->m_debug)
        cout << "random direction selected: " << dir << endl;
    }

    VID recent = ExpandTree(dir);
    if(recent != INVALID_VID) {

      //see if tree is connected to goals
      if(m_evaluateGoal)
        EvaluateGoals(recent);

      //evaluate the roadmap
      bool evalMap = this->EvaluateMap();
      mapPassedEvaluation = evalMap &&
        ((m_evaluateGoal && m_goalsNotFound.empty()) || !m_evaluateGoal);
      if(this->GetRoadmap()->GetGraph()->get_num_vertices() <=15000)
        mapPassedEvaluation = true;
      if(this->m_debug && m_goalsNotFound.empty())
        cout << "RRT FOUND ALL GOALS" << endl;
    }
  }

  stats->StopClock("RRT Generation");
  if(this->m_debug) {
    stats->PrintClock("RRT Generation", cout);
    cout<<"\nEnd Running KinodynamicRRTStrategy::" << endl;
  }
}

template<class MPTraits>
void
KinodynamicRRTStrategy<MPTraits>::
Finalize() {
  RoadmapType* rdmp = this->GetRoadmap();
  string base = this->GetBaseFilename();

  //perform query if query was given as input
  vector<VID> path;
  if(m_query) {
    if(m_evaluateGoal) {
      m_query->SetWritePath(true);
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
typename KinodynamicRRTStrategy<MPTraits>::StateType
KinodynamicRRTStrategy<MPTraits>::
GoalBiasedDirection() {
  // Determine direction, make sure goal not found
  if(m_goalsNotFound.empty())
    return StateType();
  else {
    size_t goalNum = LRand()%m_goalsNotFound.size();
    return m_goals[m_goalsNotFound[goalNum]];
  }
}

template<class MPTraits>
typename KinodynamicRRTStrategy<MPTraits>::StateType
KinodynamicRRTStrategy<MPTraits>::
SelectDirection(){
  StateType dir;
  dir.GetRandomCfg(this->GetEnvironment());
  return dir;
}

template<class MPTraits>
typename KinodynamicRRTStrategy<MPTraits>::VID
KinodynamicRRTStrategy<MPTraits>::
ExpandTree(StateType& _dir) {
  // Setup MP Variables
  StatClass* stats = this->GetStatClass();
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dm);
  NeighborhoodFinderPointer nf = this->GetNeighborhoodFinder(m_nf);
  ExtenderPointer e = this->GetExtender(m_extenderLabel);
  RoadmapType* rdmp = this->GetRoadmap();
  GraphType* g = rdmp->GetGraph();

  VID recentVID = INVALID_VID;

  // Find closest Cfg in map
  vector<pair<VID, double> > neighbors;
  vector<StateType> cfgs;

  stats->StartClock("NeighborhoodFinding");

  nf->FindNeighbors(this->GetRoadmap(), _dir, back_inserter(neighbors));

  stats->StopClock("NeighborhoodFinding");

  VID nearVID = neighbors[0].first;
  StateRef nearest = g->GetVertex(neighbors[0].first);
  StateType newCfg;
  vector<StateType> intermediates, rintermediates;

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
    typedef typename vector<StateType>::iterator CIT;
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
  if(this->m_debug)
  cout << "Expansion:: " << ++expansions << "\tto " << newCfg << endl;

  if(this->m_debug)
    cout << "RRT from " << nearVID
      << " expanded to " << newCfg << endl;

  // If good to go, add to roadmap
  if(dist >= m_minDist) {
    recentVID = g->AddVertex(newCfg);
    if(nearVID != recentVID) {
      g->GetVertex(recentVID).SetStat("Parent", neighbors[0].first);
      g->AddEdge(nearVID, recentVID, lpOutput.m_edge);
    }
    else
      recentVID = INVALID_VID;
  }
  else {
    if(this->m_debug)
      cout << "\t(expansion too close, not adding)\n";
  }

  return recentVID;
}

template<class MPTraits>
void
KinodynamicRRTStrategy<MPTraits>::
EvaluateGoals(VID _newVID) {

  DistanceMetricPointer dmp = this->GetDistanceMetric(m_dm);
  StateType& qnew = this->GetRoadmap()->GetGraph()->GetVertex(_newVID);

  // Check if goals have been found
  vector<size_t>::iterator i = m_goalsNotFound.begin();
  while(i != m_goalsNotFound.end()) {
    double dist = dmp->Distance(m_goals[*i], qnew);

    if(this->m_debug)
      cout << "Distance to goal::" << dist << endl;

    if(this->m_debug)
    cout << "dist: " << dist << endl;
    if(dist < m_goalDist) {
      if(this->m_debug)
        cout << "Goal found::" << m_goals[*i] << endl;
      m_query->GetQuery()[*i + 1] = qnew;
      i = m_goalsNotFound.erase(i);
    }
    else
      ++i;
  }
}

template<class MPTraits>
void
KinodynamicRRTStrategy<MPTraits>::
SetMPProblem(MPProblemType* _problem) {
  MPBaseObject<MPTraits>::SetMPProblem(_problem);
  if(m_query)
    m_query->SetMPProblem(_problem);
}

#endif
