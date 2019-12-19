#ifndef TOGGLE_LP_H_
#define TOGGLE_LP_H_

#include "LocalPlannerMethod.h"

#include <containers/sequential/graph/algorithms/dijkstra.h>

////////////////////////////////////////////////////////////////////////////////
/// TODO
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ToggleLP: public LocalPlannerMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ToggleLP(const string& _vcLabel = "", const string& _lpLabel = "",
        const string& _dmLabel = "",
        int _maxIter = 0, bool _saveIntermediates = false);
    ToggleLP(XMLNode& _node);
    void InitVars();
    virtual ~ToggleLP();

    void CalcStats(bool _var, bool _toggle);

    virtual void Initialize() override;
    virtual void Print(ostream& _os) const;

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

  protected:

    // Default for non closed chains - alters midpoint to a distance delta away
    // on a random ray
    template <typename Enable> CfgType ChooseAlteredCfg(
        const CfgType& _c1, const CfgType& _c2,
        typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy = 0);

    // Specialization for closed chains - choose random point
    template <typename Enable> CfgType ChooseAlteredCfg(
        const CfgType& _c1, const CfgType& _c2,
        typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy = 0);

    bool IsConnectedToggle(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    bool ToggleConnect(
        const CfgType& _s, const CfgType& _g, const CfgType& _n1, const CfgType& _n2,
        bool _toggle, LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes, int _depth = 0);

  private:
    //input
    string m_vcLabel, m_lpLabel, m_dmLabel;
    int m_maxIter;

    //needed variables for record keeping and cycle detection
    double m_iterations;
    bool m_degeneracyReached;
    deque<CfgType> m_colHist;
    std::unique_ptr<RoadmapType> m_pathGraph;
    VID m_sVID, m_gVID;
};

template<class MPTraits>
ToggleLP<MPTraits>::
ToggleLP(const string& _vclabel, const string& _lpLabel,
    const string& _dmLabel, int _maxIter, bool _saveIntermediates) :
  LocalPlannerMethod<MPTraits>(_saveIntermediates),
  m_vcLabel(_vclabel), m_lpLabel(_lpLabel), m_dmLabel(_dmLabel), m_maxIter(_maxIter) {
  InitVars();
}

template<class MPTraits>
ToggleLP<MPTraits>::
ToggleLP(XMLNode& _node) :
    LocalPlannerMethod<MPTraits>(_node) {
  InitVars();

  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Label");
  m_lpLabel = _node.Read("lpLabel", true, "", "Local Planner Label");
  m_dmLabel = _node.Read("dmLabel", true, "", "Distance Metric Label");
  m_maxIter = _node.Read("maxIter", false, 10, 0, MAX_INT,
      "Maximum number of m_iterations");
}

template<class MPTraits>
void
ToggleLP<MPTraits>::InitVars(){
  this->SetName("ToggleLP");
  m_iterations = 0;
  m_degeneracyReached = false;
  m_sVID = m_gVID = -1;
}

template<class MPTraits>
ToggleLP<MPTraits>::~ToggleLP() = default;

template<class MPTraits>
void
ToggleLP<MPTraits>::
Initialize() {
  auto robot = this->GetTask()->GetRobot();
  m_pathGraph.reset(new RoadmapType(robot));
}

template<class MPTraits>
void
ToggleLP<MPTraits>::Print(ostream& _os) const {
  LocalPlannerMethod<MPTraits>::Print(_os);
  _os << "\tmax iter =  " << m_maxIter
      << "\n\tvc label :  " << m_vcLabel
      << "\n\tlp label : " << m_lpLabel
      << "\n\tdm label : " << m_dmLabel
      << endl;
}

template<class MPTraits>
bool
ToggleLP<MPTraits>::IsConnected(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
#ifndef _PARALLEL

  StatClass* stats = this->GetStatClass();

  //Note : Initialize connected to false to avoid compiler warning in parallel
  //code.  If I am wrong please correct
  bool connected = false;
  //To do : fix me-Dijkstra doesn't compile with pGraph!!!!!!
  //clear lpOutput
  _lpOutput->Clear();
  m_pathGraph->clear();
  m_sVID = m_pathGraph->AddVertex(_c1);
  m_gVID = m_pathGraph->AddVertex(_c2);

  stats->IncLPAttempts(this->GetNameAndLabel());
  int cdCounter = 0;

  connected = IsConnectedToggle(_c1, _c2, _col, _lpOutput, cdCounter,
      _positionRes, _orientationRes, _checkCollision, _savePath);
  if(connected){
    stats->IncLPConnections(this->GetNameAndLabel());
    //find path in m_pathGraph
    vector<VID> path;
    stapl::sequential::find_path_dijkstra(*m_pathGraph, m_sVID, m_gVID, path,
        WeightType::MaxWeight());
    if(path.size() > 0) {
      for(size_t i = 1; i <path.size() - 1; i++) {
        _lpOutput->m_intermediates.push_back(m_pathGraph->find_vertex(path[i])->property());
      }
    }
    _lpOutput->AddIntermediatesToWeights(this->m_saveIntermediates);
    _lpOutput->SetLPLabel(this->GetLabel());
  }

  stats->IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  return connected;
#else
  stapl_assert(false, "ToggleLP calling Dijkstra on pGraph");
  return false;
#endif
}

// Default for non closed chains - alters midpoint to a distance delta away on a
// random ray
template<class MPTraits>
template <typename Enable>
typename MPTraits::CfgType
ToggleLP<MPTraits>::ChooseAlteredCfg(
    const CfgType& _c1, const CfgType& _c2,
    typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy){
  Environment* env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();
  auto dm = this->GetDistanceMetric(m_dmLabel);
  StatClass* stats = this->GetStatClass();

  size_t attempts = 0;
  CfgType mid(robot), temp(robot);
  mid = (_c1 + _c2)/2.0;
  do{
    CfgType incr(robot);
    double dist = dm->Distance(_c1, _c2) * sqrt(2.0)/2.0;
    incr.GetRandomRay(dist, dm);
    temp = incr + mid;
  } while(!temp.InBounds(env) && attempts++ < 10);
  if(attempts == 10){
    stats->IncStat("Toggle::MaxAttemptsForRay", 1);
    /// @TODO Should this cfg have robot also?
    return CfgType(robot);
  }
  return temp;
}

// Specialization for closed chains - choose random point
template<class MPTraits>
template <typename Enable>
typename MPTraits::CfgType
ToggleLP<MPTraits>::ChooseAlteredCfg(
    const CfgType& _c1, const CfgType& _c2,
    typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy) {
  Environment* env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();

  CfgType temp(robot);
  do {
    temp.GetRandomCfg(env);
  } while(!temp.InBounds(env));
  return temp;
}

template<class MPTraits>
bool
ToggleLP<MPTraits>::IsConnectedToggle(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
  StatClass* stats = this->GetStatClass();

  m_iterations = 0;
  m_degeneracyReached = false;
  m_colHist.clear();

  stats->IncStat("Toggle::TotalLP");

  if(this->m_debug) {
    cout << "Total LP::" << stats->GetStat("Toggle::TotalLP") << endl
         << "ToggleLP LP::" << "\n\t" << _c1 << "\n\t" << _c2 << endl;
  }

  string callee(this->GetNameAndLabel()+"::IsConnectedToggle");
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto lp = this->GetLocalPlanner(m_lpLabel);

  if(lp->IsConnected(_c1, _c2, _col, _lpOutput,
      _positionRes, _orientationRes, _checkCollision, _savePath))
    return true;

  auto robot = this->GetTask()->GetRobot();
  if(_col == CfgType(robot))
    return false;
  if(this->m_debug)
    cout << "col::" << _col << endl;

  stats->IncStat("Toggle::TotalCalls");

  CfgType temp = ChooseAlteredCfg<CfgType>(_c1, _c2);
  CfgType n = temp;
  if(n == CfgType(robot))
    return false;

  bool isValid = vc->IsValid(n, callee);
  _cdCounter++;

  if(this->m_debug) {
    cout << "N is " << isValid << "::" << n << endl;
    VDClearAll();
    VDComment("Start Local Plan");
    VDAddNode(_c1);
    VDAddNode(_c2);
    VDAddTempCfg(_c1, true);
    VDAddTempCfg(_c2, true);
    VDAddTempCfg(n, isValid);
    VDAddTempCfg(_col, false);
  }
  if(isValid) {
    VID nvid = m_pathGraph->AddVertex(n);
    CfgType c2(robot), c3(robot);
    bool b1 = lp->IsConnected(_c1, n, c2, _lpOutput,
        _positionRes, _orientationRes, true, false);
    if(b1)
      m_pathGraph->AddEdge(m_sVID, nvid, pair<WeightType, WeightType>());
    bool b2 = lp->IsConnected(_c2, n, c3, _lpOutput,
        _positionRes, _orientationRes, true, false);
    if(b2)
      m_pathGraph->AddEdge(m_gVID, nvid, pair<WeightType, WeightType>());
    if(this->m_debug) {
      VDAddNode(n);
      VDAddTempEdge(_c1, n);
      VDAddTempEdge(_c2, n);
      VDAddTempCfg(c2, false);
      VDAddTempCfg(c3, false);
      cout << "n-c2::" << b1 << "::" << c2 << endl
           << "n-c3::" << b2 << "::" << c3 << endl;
    }
    if(!b1 && c2 != CfgType(robot))
      b1 = ToggleConnect(_col, c2, _c1, n, false, _lpOutput,
          _positionRes, _orientationRes);
    else if(this->m_debug && b1)
      VDAddEdge(_c1, n);
    if(!b2 && c3 != CfgType(robot) && b1)
      b2 = ToggleConnect(_col, c3, _c2, n, false, _lpOutput,
          _positionRes, _orientationRes);
    else if(this->m_debug && b2)
      VDAddEdge(_c2, n);
    if(b1 && b2 && this->m_debug) {
      VDQuery(_c1, _c2);
      VDClearAll();
      VDAddTempCfg(_c1, true);
      VDAddTempCfg(_c2, true);
      VDAddTempCfg(_col, false);
      VDAddTempCfg(n, isValid);
    }
    CalcStats(b1 && b2, true);
    return b1 && b2;
  }
  else{
    bool b = ToggleConnect(_col, n, _c1, _c2, false, _lpOutput,
        _positionRes, _orientationRes);
    if(b && this->m_debug) {
      VDQuery(_c1, _c2);
      VDClearAll();
      VDAddTempCfg(_c1, true);
      VDAddTempCfg(_c2, true);
      VDAddTempCfg(_col, false);
      VDAddTempCfg(n, isValid);
    }
    CalcStats(b, false);
    return b;
  }
};

template<class MPTraits>
void
ToggleLP<MPTraits>::CalcStats(bool _val, bool _toggle) {
  StatClass* stats = this->GetStatClass();

  if(_val) {
    if(_toggle)
      stats->IncStat("Toggle::FreeSuccess");
    else
      stats->IncStat("Toggle::CollisionSuccess");
    stats->AddToHistory("Toggle::IterationSuccess", m_iterations);
  }
  else {
    if(_toggle)
      stats->IncStat("Toggle::FreeFailure");
    else
      stats->IncStat("Toggle::CollisionFailure");
    stats->AddToHistory("Toggle::IterationFailure", m_iterations);
    if(m_degeneracyReached) {
      stats->IncStat("Toggle::DegenerateFailure");
      stats->IncStat("Toggle::DegenerateFailureIter", m_iterations);
    }
    else {
      stats->IncStat("Toggle::BlockingFailure");
      stats->IncStat("Toggle::BlockingFailureIter", m_iterations);
    }
  }
  stats->AddToHistory("Toggle::Iteration", m_iterations);
  stats->IncStat("Toggle::TotalIterations", m_iterations);
  if(m_iterations > stats->GetStat("Toggle::MaxIterations"))
    stats->SetStat("Toggle::MaxIterations", m_iterations);


  double freeSuccess = stats->GetStat("Toggle::FreeSuccess");
  double collisionSuccess = stats->GetStat("Toggle::CollisionSuccess");
  double freeFailure = stats->GetStat("Toggle::FreeFailure");
  double collisionFailure = stats->GetStat("Toggle::CollisionFailure");
  double degenerateFailure = stats->GetStat("Toggle::DegenerateFailure");
  double blockingFailure = stats->GetStat("Toggle::BlockingFailure");
  stats->SetStat("Toggle::FreeSuccess%", freeSuccess / (freeSuccess + freeFailure));
  stats->SetStat("Toggle::CollisionSuccess%",
      collisionSuccess / (collisionSuccess + collisionFailure));
  stats->SetStat("Toggle::TotalSuccess%", (freeSuccess + collisionSuccess) /
      (collisionSuccess + collisionFailure + freeSuccess + freeFailure));
  stats->SetStat("Toggle::IterAvg",
      stats->GetStat("Toggle::TotalIterations") / stats->GetStat("Toggle::TotalCalls"));
  stats->SetStat("Toggle::DegeneracyFailure%",
      degenerateFailure / (collisionFailure + freeFailure));
  stats->SetStat("Toggle::BlockingFailure%",
      blockingFailure / (collisionFailure + freeFailure));
  stats->SetStat("Toggle::DegenerateIterAvg",
      stats->GetStat("Toggle::DegenerateFailureIter") / degenerateFailure);
  stats->SetStat("Toggle::BlockingIterAvg",
      stats->GetStat("Toggle::BlockingFailureIter") / blockingFailure);
}

template<class MPTraits>
bool
ToggleLP<MPTraits>::ToggleConnect(
    const CfgType& _s, const CfgType& _g, const CfgType& _n1, const CfgType& _n2,
    bool _toggle, LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes, int _depth) {

  if(_depth > m_maxIter) return false;

  ++m_iterations;

  if(this->m_debug)
    cout << "ToggleConnect::" << _toggle
         << "\n\ts::" << _s
         << "\n\tg::" << _g
         << "\n\tn1::" << _n1
         << "\n\tn2::" << _n2
         << endl;

  //set up variables for VC and LP
  auto lp = this->GetLocalPlanner(m_lpLabel);

  if(this->m_debug) VDAddTempEdge(_s, _g);

  //check connection between source and goal
  auto robot = this->GetTask()->GetRobot();
  CfgType c(robot); // collision CfgType
  if(!_toggle)
    this->GetValidityChecker(m_vcLabel)->ToggleValidity();
  bool connect = lp->IsConnected(_s, _g, c, _lpOutput,
      _positionRes, _orientationRes, true, false);
  if(!_toggle)
    this->GetValidityChecker(m_vcLabel)->ToggleValidity();

  if(this->m_debug) cout << "connect::" << connect << "\n\tc::" << c << endl;

  //c is outside of the bounding box, degenerate case
  if(!connect && c == CfgType(robot)) return false;

  //successful connection, add the edge and return validity state
  if(connect) {
    if(_toggle) {
      m_pathGraph->AddEdge(m_pathGraph->GetVID(_s), m_pathGraph->GetVID(_g), pair<WeightType,
          WeightType>());
    }
    if(this->m_debug && _toggle)
      VDAddEdge(_s, _g);
    return _toggle;
  }

  if(!_toggle) {
    m_pathGraph->AddVertex(c);
  }

  if(this->m_debug) VDAddTempCfg(c, !_toggle);
  if(this->m_debug && !_toggle) VDAddNode(c);

  //look for degeneracies
  if(_n1 == c || _n2 == c) {
    m_degeneracyReached = true;
    return false;
  }
  typedef typename deque<CfgType>::iterator CIT;
  for(CIT cit = m_colHist.begin(); cit != m_colHist.end(); cit++) {
    if(c == *cit) {
      m_degeneracyReached = true;
      return false;
    }
  }
  if(m_colHist.size() >= 50) m_colHist.pop_back();
  m_colHist.push_front(c);

  //recurse
  if(!_toggle)
    return ToggleConnect(_n1, c, _s, _g, !_toggle, _lpOutput,
        _positionRes, _orientationRes, _depth+1) && ToggleConnect(_n2, c, _s,
          _g, !_toggle, _lpOutput, _positionRes, _orientationRes, _depth+1);
  else
    return ToggleConnect(_n1, c, _s, _g, !_toggle, _lpOutput,
        _positionRes, _orientationRes, _depth+1) || ToggleConnect(_n2, c, _s,
          _g, !_toggle, _lpOutput, _positionRes, _orientationRes, _depth+1);
}

#endif
