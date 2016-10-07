#ifndef TOGGLE_LP_H_
#define TOGGLE_LP_H_

#include "LocalPlannerMethod.h"

#include <containers/sequential/graph/algorithms/dijkstra.h>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ToggleLP: public LocalPlannerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;

    ToggleLP(const string& _vcLabel = "", const string& _lpLabel = "",
        const string& _dmLabel = "",
        int _maxIter = 0, bool _saveIntermediates = false);
    ToggleLP(MPProblemType* _problem, XMLNode& _node);
    void InitVars();
    virtual ~ToggleLP();

    void CalcStats(bool _var, bool _toggle);
    virtual void Print(ostream& _os) const;

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    virtual vector<CfgType> ReconstructPath(
        const CfgType& _c1, const CfgType& _c2,
        const vector<CfgType>& _intermediates,
        double _posRes, double _oriRes);

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
    GraphType m_pathGraph;
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
ToggleLP(MPProblemType* _problem, XMLNode& _node) :
    LocalPlannerMethod<MPTraits>(_problem, _node) {
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
ToggleLP<MPTraits>::~ToggleLP() {
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

  StatClass* stats = this->GetMPProblem()->GetStatClass();

  //Note : Initialize connected to false to avoid compiler warning in parallel
  //code.  If I am wrong please correct
  bool connected = false;
  //To do : fix me-Dijkstra doesn't compile with pGraph!!!!!!
  //clear lpOutput
  _lpOutput->Clear();
  m_pathGraph.clear();
  m_sVID = m_pathGraph.AddVertex(_c1);
  m_gVID = m_pathGraph.AddVertex(_c2);

  stats->IncLPAttempts(this->GetNameAndLabel());
  int cdCounter = 0;

  connected = IsConnectedToggle(_c1, _c2, _col, _lpOutput, cdCounter,
      _positionRes, _orientationRes, _checkCollision, _savePath);
  if(connected){
    stats->IncLPConnections(this->GetNameAndLabel());
    //find path in m_pathGraph
    vector<VID> path;
    stapl::sequential::find_path_dijkstra(m_pathGraph, m_sVID, m_gVID, path,
        WeightType::MaxWeight());
    if(path.size() > 0) {
      for(size_t i = 1; i <path.size() - 1; i++) {
        _lpOutput->m_intermediates.push_back(m_pathGraph.find_vertex(path[i])->property());
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
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  size_t attempts = 0;
  CfgType mid, temp;
  mid = (_c1 + _c2)/2.0;
  do{
    CfgType incr;
    double dist = dm->Distance(_c1, _c2) * sqrt(2.0)/2.0;
    incr.GetRandomRay(dist, dm);
    temp = incr + mid;
  } while(!env->InBounds(temp) && attempts++ < 10);
  if(attempts == 10){
    stats->IncLPStat("Toggle::MaxAttemptsForRay", 1);
    return CfgType();
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
  Environment* env = this->GetMPProblem()->GetEnvironment();

  CfgType temp;
  do{
    temp.GetRandomCfg(env);
  } while(!env->InBounds(temp));
  return temp;
}

template<class MPTraits>
bool
ToggleLP<MPTraits>::IsConnectedToggle(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  m_iterations = 0;
  m_degeneracyReached = false;
  m_colHist.clear();

  stats->IncLPStat("Toggle::TotalLP");

  if(this->m_debug) {
    cout << "Total LP::" << stats->GetLPStat("Toggle::TotalLP") << endl
         << "ToggleLP LP::" << "\n\t" << _c1 << "\n\t" << _c2 << endl;
  }

  string callee(this->GetNameAndLabel()+"::IsConnectedToggle");
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);

  if(lp->IsConnected(_c1, _c2, _col, _lpOutput,
      _positionRes, _orientationRes, _checkCollision, _savePath))
    return true;

  if(_col == CfgType())
    return false;
  if(this->m_debug)
    cout << "col::" << _col << endl;

  stats->IncLPStat("Toggle::TotalCalls");

  CfgType temp = ChooseAlteredCfg<CfgType>(_c1, _c2);
  CfgType n = temp;
  if(n == CfgType())
    return false;

  bool isValid = vc->IsValid(n, callee);
  _cdCounter++;

  if(this->m_debug){
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
  if(isValid){
    VID nvid = m_pathGraph.AddVertex(n);
    CfgType c2, c3;
    bool b1 = lp->IsConnected(_c1, n, c2, _lpOutput,
        _positionRes, _orientationRes, true, false);
    if(b1)
      m_pathGraph.AddEdge(m_sVID, nvid, pair<WeightType, WeightType>());
    bool b2 = lp->IsConnected(_c2, n, c3, _lpOutput,
        _positionRes, _orientationRes, true, false);
    if(b2)
      m_pathGraph.AddEdge(m_gVID, nvid, pair<WeightType, WeightType>());
    if(this->m_debug) {
      VDAddNode(n);
      VDAddTempEdge(_c1, n);
      VDAddTempEdge(_c2, n);
      VDAddTempCfg(c2, false);
      VDAddTempCfg(c3, false);
      cout << "n-c2::" << b1 << "::" << c2 << endl
           << "n-c3::" << b2 << "::" << c3 << endl;
    }
    if(!b1 && c2 != CfgType())
      b1 = ToggleConnect(_col, c2, _c1, n, false, _lpOutput,
          _positionRes, _orientationRes);
    else if(this->m_debug && b1)
      VDAddEdge(_c1, n);
    if(!b2 && c3 != CfgType() && b1)
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
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  if(_val) {
    if(_toggle)
      stats->IncLPStat("Toggle::FreeSuccess");
    else
      stats->IncLPStat("Toggle::CollisionSuccess");
    stats->AddToHistory("Toggle::IterationSuccess", m_iterations);
  }
  else {
    if(_toggle)
      stats->IncLPStat("Toggle::FreeFailure");
    else
      stats->IncLPStat("Toggle::CollisionFailure");
    stats->AddToHistory("Toggle::IterationFailure", m_iterations);
    if(m_degeneracyReached) {
      stats->IncLPStat("Toggle::DegenerateFailure");
      stats->IncLPStat("Toggle::DegenerateFailureIter", m_iterations);
    }
    else {
      stats->IncLPStat("Toggle::BlockingFailure");
      stats->IncLPStat("Toggle::BlockingFailureIter", m_iterations);
    }
  }
  stats->AddToHistory("Toggle::Iteration", m_iterations);
  stats->IncLPStat("Toggle::TotalIterations", m_iterations);
  if(m_iterations > stats->GetLPStat("Toggle::MaxIterations"))
    stats->SetLPStat("Toggle::MaxIterations", m_iterations);


  double freeSuccess = stats->GetLPStat("Toggle::FreeSuccess");
  double collisionSuccess = stats->GetLPStat("Toggle::CollisionSuccess");
  double freeFailure = stats->GetLPStat("Toggle::FreeFailure");
  double collisionFailure = stats->GetLPStat("Toggle::CollisionFailure");
  double degenerateFailure = stats->GetLPStat("Toggle::DegenerateFailure");
  double blockingFailure = stats->GetLPStat("Toggle::BlockingFailure");
  stats->SetLPStat("Toggle::FreeSuccess%", freeSuccess / (freeSuccess + freeFailure));
  stats->SetLPStat("Toggle::CollisionSuccess%",
      collisionSuccess / (collisionSuccess + collisionFailure));
  stats->SetLPStat("Toggle::TotalSuccess%", (freeSuccess + collisionSuccess) /
      (collisionSuccess + collisionFailure + freeSuccess + freeFailure));
  stats->SetLPStat("Toggle::IterAvg",
      stats->GetLPStat("Toggle::TotalIterations") / stats->GetLPStat("Toggle::TotalCalls"));
  stats->SetLPStat("Toggle::DegeneracyFailure%",
      degenerateFailure / (collisionFailure + freeFailure));
  stats->SetLPStat("Toggle::BlockingFailure%",
      blockingFailure / (collisionFailure + freeFailure));
  stats->SetLPStat("Toggle::DegenerateIterAvg",
      stats->GetLPStat("Toggle::DegenerateFailureIter") / degenerateFailure);
  stats->SetLPStat("Toggle::BlockingIterAvg",
      stats->GetLPStat("Toggle::BlockingFailureIter") / blockingFailure);
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
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);

  if(this->m_debug) VDAddTempEdge(_s, _g);

  //check connection between source and goal
  CfgType c; // collision CfgType
  if(!_toggle)
    this->GetMPProblem()->GetValidityChecker(m_vcLabel)->ToggleValidity();
  bool connect = lp->IsConnected(_s, _g, c, _lpOutput,
      _positionRes, _orientationRes, true, false);
  if(!_toggle)
    this->GetMPProblem()->GetValidityChecker(m_vcLabel)->ToggleValidity();

  if(this->m_debug) cout << "connect::" << connect << "\n\tc::" << c << endl;

  //c is outside of the bounding box, degenerate case
  if(!connect && c == CfgType()) return false;

  //successful connection, add the edge and return validity state
  if(connect) {
    if(_toggle) {
      m_pathGraph.AddEdge(m_pathGraph.GetVID(_s), m_pathGraph.GetVID(_g), pair<WeightType,
          WeightType>());
    }
    if(this->m_debug && _toggle)
      VDAddEdge(_s, _g);
    return _toggle;
  }

  if(!_toggle) {
    m_pathGraph.AddVertex(c);
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

template<class MPTraits>
vector<typename MPTraits::CfgType>
ToggleLP<MPTraits>::ReconstructPath(
    const CfgType& _c1, const CfgType& _c2,
    const vector<CfgType>& _intermediates,
    double _posRes, double _oriRes) {
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);
  LPOutput<MPTraits>* lpOutput = new LPOutput<MPTraits>();
  LPOutput<MPTraits>* dummyLPOutput = new LPOutput<MPTraits>();
  CfgType col;
  if(_intermediates.size() > 0) {
    lp->IsConnected(_c1, _intermediates[0], col,
        dummyLPOutput, _posRes, _oriRes, false, true);
    for(size_t j = 0; j < dummyLPOutput->m_path.size(); j++)
      lpOutput->m_path.push_back(dummyLPOutput->m_path[j]);
    for(size_t i = 0; i < _intermediates.size() - 1; i++) {
      lpOutput->m_path.push_back(_intermediates[i]);
      lp->IsConnected(_intermediates[i], _intermediates[i + 1],
          col, dummyLPOutput, _posRes, _oriRes, false, true);
      for(size_t j = 0; j < dummyLPOutput->m_path.size(); j++)
        lpOutput->m_path.push_back(dummyLPOutput->m_path[j]);
    }
    lpOutput->m_path.push_back(_intermediates[_intermediates.size() - 1]);
    lp->IsConnected(_intermediates[_intermediates.size() - 1],
        _c2, col, dummyLPOutput, _posRes, _oriRes, false, true);
    for(size_t j = 0; j < dummyLPOutput->m_path.size(); j++)
      lpOutput->m_path.push_back(dummyLPOutput->m_path[j]);
  }
  else {
    lp->IsConnected(_c1, _c2, col, dummyLPOutput,
        _posRes, _oriRes, false, true);
    for(size_t j = 0; j < dummyLPOutput->m_path.size(); j++)
      lpOutput->m_path.push_back(dummyLPOutput->m_path[j]);
  }
  vector<CfgType> path = lpOutput->m_path;
  delete lpOutput;
  delete dummyLPOutput;
  return path;
}

#endif
