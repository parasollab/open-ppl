#ifndef KINODYNAMIC_RRT_STRATEGY_H_
#define KINODYNAMIC_RRT_STRATEGY_H_

#include "MPStrategyMethod.h"
#include "MapEvaluators/RRTQuery.h"

#ifdef VIZMO
#include "Models/Vizmo.h"
#endif

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

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType StateType;
    typedef typename MPTraits::CfgRef StateRef;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;

    ///@}
    ///\name Construction
    ///@{

    KinodynamicRRTStrategy(const StateType& _start = StateType(),
        const StateType& _goal = StateType(), string _dm = "", string _nf = "",
        string _vc= "", string _extenderLabel = "",
        vector<string> _evaluators = vector<string>(),
        double _goalDist = 10.0, double _minDist = 0.01,
        double _growthFocus = 0.05, bool _evaluateGoal = true);

    KinodynamicRRTStrategy(MPProblemType* _problem, XMLNode& _node);

    virtual ~KinodynamicRRTStrategy() = default;

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const override;

    ///@}
    ///\name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Run() override;
    virtual void Finalize() override;

    ///@}

  protected:

    ///\name Helper Functions
    ///@{

    StateType SelectDirection();
    virtual VID ExpandTree(StateType& _dir);

    ///@}
    ///\name Internal State
    ///@{

    string m_dmLabel;
    string m_nfLabel;
    string m_vcLabel;
    string m_extenderLabel;
    RRTQuery<MPTraits>* m_query{nullptr};
    double m_growthFocus;

    ///}
};


template<class MPTraits>
KinodynamicRRTStrategy<MPTraits>::
KinodynamicRRTStrategy(const StateType& _start, const StateType& _goal,
    string _dm, string _nf, string _vc,
    string _extenderLabel, vector<string> _evaluators,
    double _goalDist, double _minDist, double _growthFocus, bool _evaluateGoal) :
    m_dmLabel(_dm), m_nfLabel(_nf), m_vcLabel(_vc),
    m_extenderLabel(_extenderLabel),
    m_growthFocus(_growthFocus) {
  this->SetName("KinodynamicRRTStrategy");
  this->m_meLabels = _evaluators;
}


template<class MPTraits>
KinodynamicRRTStrategy<MPTraits>::
KinodynamicRRTStrategy(MPProblemType* _problem, XMLNode& _node) :
    MPStrategyMethod<MPTraits>(_problem, _node) {
  this->SetName("KinodynamicRRTStrategy");
  ParseXML(_node);
}


template<class MPTraits>
void
KinodynamicRRTStrategy<MPTraits>::
ParseXML(XMLNode& _node) {
  for(auto& child : _node)
    if(child.Name() == "Evaluator")
      this->m_meLabels.push_back(child.Read("label", true, "",
          "Evaluation Method"));

  m_growthFocus = _node.Read("growthFocus", false, 0.0, 0.0, 1.0,
      "#GeneratedTowardsGoal/#Generated");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
  m_dmLabel = _node.Read("dmLabel", true, "", "Distance Metric");
  m_extenderLabel = _node.Read("extenderLabel", true, "", "Extender label");
}

template<class MPTraits>
void
KinodynamicRRTStrategy<MPTraits>::
Print(ostream& _os) const {
  _os << "KinodynamicRRTStrategy::Print" << endl;
  _os << "\tNeighborhood Finder:: " << m_nfLabel << endl;
  _os << "\tDistance Metric:: " << m_dmLabel << endl;
  _os << "\tValidity Checker:: " << m_vcLabel << endl;
  _os << "\tExtender:: " << m_extenderLabel << endl;
  _os << "\tEvaluators:: " << endl;
  for(auto&  s : this->m_meLabels)
    _os << "\t\t" << s << endl;
  _os << "\tgrowth focus:: " << m_growthFocus << endl;
}

template<class MPTraits>
void
KinodynamicRRTStrategy<MPTraits>::
Initialize() {
  GraphType* g = this->GetRoadmap()->GetGraph();

  // Check for query info.
  try {
    string label = "RRTQuery";
    m_query = static_cast<RRTQuery<MPTraits>*>(this->GetMapEvaluator(label).
        get());
    auto iter = find(this->m_meLabels.begin(), this->m_meLabels.end(), label);
    if(iter == this->m_meLabels.end())
      this->m_meLabels.push_back(label);
  }
  catch(RunTimeException) {
    m_query = nullptr;
  }


  if(!m_query->GetQuery().empty())
    // A query was given: add the root vertex to the roadmap.
    g->AddVertex(m_query->GetQuery().front());
  else {
    // No query was given, so make a start goal and grow.
    Environment* env = this->GetEnvironment();
    auto vc = this->GetValidityChecker(m_vcLabel);
    const string callee = "KinodynamicRRTStrategy::RRT";

    StateType tmp;
    do {
      tmp.GetRandomCfg(env);
    } while(!env->InBounds(tmp) || !vc->IsValid(tmp, callee));
    g->AddVertex(tmp);
  }

  if(this->m_debug)
    cout << "\nRunning KinodynamicRRTStrategy::" << endl;

  this->GetStatClass()->StartClock("RRT Generation");
}


template<class MPTraits>
void
KinodynamicRRTStrategy<MPTraits>::
Run() {
  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation) {
    //find my growth direction. Default is too randomly select node or bias
    //towards a goal
    StateType dir;
    if(m_query && DRand() < m_growthFocus && !m_query->GetGoals().empty()) {
      dir = m_query->GetRandomGoal();
      if(this->m_debug)
        cout << "goal biased direction selected: " << dir << endl;
    }
    else {
      dir = SelectDirection();
      if(this->m_debug)
        cout << "random direction selected: " << dir << endl;
    }

    if(ExpandTree(dir) != INVALID_VID) {
      // A new node was made. Re-evaluate the roadmap.
      mapPassedEvaluation = this->EvaluateMap();
    }
#ifdef VIZMO
    GetVizmo().GetMap()->RefreshMap();
#endif
  }
}


template<class MPTraits>
void
KinodynamicRRTStrategy<MPTraits>::
Finalize() {
  StatClass* stats = this->GetStatClass();
  stats->StopClock("RRT Generation");

  if(this->m_debug) {
    stats->PrintClock("RRT Generation", cout);
    cout << "\nEnd Running KinodynamicRRTStrategy::" << endl;
  }

  RoadmapType* rdmp = this->GetRoadmap();
  string base = this->GetBaseFilename();

  //perform query if query was given as input
  if(m_query && !m_query->GetQuery().empty() && m_query->GetGoals().empty())
    m_query->WritePath();

  //output final map
  rdmp->Write(base + ".map", this->GetEnvironment());

  //output stats
  string str = base + ".stat";
  ofstream osStat(str.c_str());
  stats->PrintAllStats(osStat, rdmp);
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
  RoadmapType* rdmp = this->GetRoadmap();
  GraphType* g = rdmp->GetGraph();
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  auto e = this->GetExtender(m_extenderLabel);

  VID recentVID = INVALID_VID;

  // Find closest Cfg in map
  stats->StartClock("NeighborhoodFinding");
  vector<pair<VID, double>> neighbors;
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
  if(dist >= e->GetMinDistance()) {
    recentVID = g->AddVertex(newCfg);
    if(nearVID != recentVID) {
      g->GetVertex(recentVID).SetStat("Parent", neighbors[0].first);
      g->AddEdge(nearVID, recentVID, lpOutput.m_edge.first);
    }
    else
      recentVID = INVALID_VID;
  }
  else if(this->m_debug)
    cout << "\t(expansion too close, not adding)\n";

  return recentVID;
}

#endif
