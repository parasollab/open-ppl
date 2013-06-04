#ifndef TOGGLELP_H_
#define TOGGLELP_H_

#include "LocalPlannerMethod.h"

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

    ToggleLP(string _vc = "", string _lp = "", int _maxIter = 0);
    ToggleLP(MPProblemType* _problem, XMLNodeReader& _node);
    void InitVars();
    virtual ~ToggleLP();

    void CalcStats(StatClass& _stats, bool _var, bool _toggle);
    virtual void PrintOptions(ostream& _os);

    virtual bool IsConnected(Environment* _env, StatClass& _stats,
        DistanceMetricPointer _dm, const CfgType& _c1, const CfgType& _c2, CfgType& _col, 
        LPOutput<MPTraits>* _lpOutput, double _positionRes, double _orientationRes,
        bool _checkCollision=true, bool _savePath=false, bool _saveFailedPath=false);

    virtual vector<CfgType> ReconstructPath(Environment* _env, DistanceMetricPointer _dm, 
        const CfgType& _c1, const CfgType& _c2, const vector<CfgType>& _intermediates, double _posRes, double _oriRes);
  protected:

    // Default for non closed chains - alters midpoint to a distance delta away on a random ray
    template <typename Enable>
      CfgType ChooseAlteredCfg(Environment* _env, StatClass& _stats,
          DistanceMetricPointer _dm,
          const CfgType& _c1, const CfgType& _c2,
          typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy = 0);

    // Specialization for closed chains - choose random point
    template <typename Enable>
      CfgType ChooseAlteredCfg(Environment* _env, StatClass& _stats,
          DistanceMetricPointer _dm,
          const CfgType& _c1, const CfgType& _c2,
          typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy = 0);

    bool IsConnectedToggle(Environment* _env, StatClass& _stats,
        DistanceMetricPointer _dm, const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, int& _cdCounter, double _positionRes, double _orientationRes,
        bool _checkCollision=true, bool _savePath=false, bool _saveFailedPath=false);

    bool ToggleConnect(Environment* _env, StatClass& _stats, 
        DistanceMetricPointer _dm, const CfgType& _s, const CfgType& _g, const CfgType& _n1, const CfgType& _n2, bool _toggle,
        LPOutput<MPTraits>* _lpOutput, double _positionRes, double _orientationRes, int _depth=0);

  private:
    //input
    string m_vcLabel, m_lpLabel;
    int m_maxIter;

    //needed variables for record keeping and cycle detection
    double m_iterations;
    bool m_degeneracyReached;
    deque<CfgType> m_colHist;
    GraphType pathGraph;
    VID svid, gvid;
};

template<class MPTraits>
ToggleLP<MPTraits>::ToggleLP(string _vc, string _lp, int _maxIter) : 
  m_vcLabel(_vc), m_lpLabel(_lp), m_maxIter(_maxIter) {
    InitVars();
  }

template<class MPTraits>
ToggleLP<MPTraits>::ToggleLP(MPProblemType* _problem, XMLNodeReader& _node) : LocalPlannerMethod<MPTraits>(_problem, _node) {
  InitVars();

  m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
  m_lpLabel = _node.stringXMLParameter("lpLabel", true, "", "Local Planner Method");
  m_maxIter = _node.numberXMLParameter("maxIter", false, 10, 0, MAX_INT, "Maximum number of m_iterations");

  _node.warnUnrequestedAttributes();
}

template<class MPTraits>
void 
ToggleLP<MPTraits>::InitVars(){
  this->SetName("ToggleLP");
  m_iterations = 0;
}

template<class MPTraits>
ToggleLP<MPTraits>::~ToggleLP() {
}

template<class MPTraits>
void
ToggleLP<MPTraits>::PrintOptions(ostream& _os) {
  _os << "    " << this->GetName() << "::  ";
  _os << "maxIter =  " << m_maxIter << " ";
  _os << "vc =  " << m_vcLabel << " ";
  _os << "lp =  " << m_lpLabel << " ";
  _os << endl;
}

template<class MPTraits>
bool 
ToggleLP<MPTraits>::IsConnected(Environment* _env, StatClass& _stats,
    DistanceMetricPointer _dm, const CfgType& _c1, const CfgType& _c2, CfgType& _col, 
    LPOutput<MPTraits>* _lpOutput, double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) {
  //Note : Initialize connected to false to avoid compiler warning in parallel code
  //If I am wrong please correct
  bool connected = false;
  ///To do : fix me-Dijkstra doesn't compile with pGraph!!!!!!
#ifndef _PARALLEL
  //clear lpOutput
  _lpOutput->Clear();
  pathGraph.clear();
  svid = pathGraph.AddVertex(_c1);
  gvid = pathGraph.AddVertex(_c2);

  _stats.IncLPAttempts(this->GetNameAndLabel());
  int cdCounter = 0; 

  connected = IsConnectedToggle(_env, _stats, _dm, _c1, _c2, _col, 
      _lpOutput, cdCounter, _positionRes, _orientationRes, 
      _checkCollision, _savePath, _saveFailedPath);
  if(connected){
    _stats.IncLPConnections(this->GetNameAndLabel());
    //find path in pathGraph
    vector<VID> path;
    stapl::sequential::find_path_dijkstra(pathGraph, svid, gvid, path, WeightType::MaxWeight());
    if(path.size()>0){
      for(size_t i = 1; i<path.size()-1; i++){
        _lpOutput->intermediates.push_back(pathGraph.find_vertex(path[i])->property());
      }
    }
    _lpOutput->AddIntermediatesToWeights(this->m_saveIntermediates);
    _lpOutput->SetLPLabel(this->GetLabel());
  }

  _stats.IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
#else
  stapl_assert(false, "ToggleLP calling Dijkstra on pGraph");
#endif
  return connected;
}

// Default for non closed chains - alters midpoint to a distance delta away on a random ray
template<class MPTraits>
template <typename Enable>
typename MPTraits::CfgType
ToggleLP<MPTraits>::ChooseAlteredCfg(Environment* _env, StatClass& _stats,
    DistanceMetricPointer _dm, const CfgType& _c1, const CfgType& _c2,
    typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy){
  size_t attempts = 0;
  CfgType mid, temp;
  mid = (_c1 + _c2)/2.0;
  do{
    CfgType incr;
    double dist = _dm->Distance(_env, _c1, _c2) * sqrt(2.0)/2.0;
    incr.GetRandomRay(dist, _env, _dm);
    temp = incr + mid;
  }while(!_env->InBounds(temp) && attempts++<10);
  if(attempts==10){ 
    _stats.IncLPStat("Toggle::MaxAttemptsForRay", 1);
    return CfgType();
  }
  return temp;
}

// Specialization for closed chains - choose random point
template<class MPTraits>
template <typename Enable>
typename MPTraits::CfgType
ToggleLP<MPTraits>::ChooseAlteredCfg(Environment* _env, StatClass& _stats,
    DistanceMetricPointer _dm, 
    const CfgType& _c1, const CfgType& _c2,
    typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy){
  CfgType temp;
  do{
    temp.GetRandomCfg(_env);
  }while(!_env->InBounds(temp));
  return temp;
}

template<class MPTraits>
bool
ToggleLP<MPTraits>::IsConnectedToggle(Environment* _env, StatClass& _stats,
    DistanceMetricPointer _dm, const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput, int& _cdCounter, double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) { 

  m_iterations=0;
  m_degeneracyReached=false;
  m_colHist.clear();

  if(this->m_recordKeep) _stats.IncLPStat("Toggle::TotalLP", 1);

  if(this->m_debug) {
    cout<<"Total LP::"<<_stats.GetLPStat("Toggle::TotalLP")<<endl;
    cout<<"ToggleLP LP::"<<"\n\t"<<_c1<<"\n\t"<<_c2<<endl;
  }

  string Callee(this->GetNameAndLabel()+"::IsConnectedToggle");
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  LocalPlannerPointer lpMethod = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);

  if(lpMethod->IsConnected(_env, _stats, _dm, _c1, _c2, _col, _lpOutput, _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath)){
    return true;
  }
  if(_col == CfgType())
    return false;
  if(this->m_debug) cout<<"col::"<<_col<<endl;

  if(this->m_recordKeep) _stats.IncLPStat("Toggle::TotalCalls", 1);

  CDInfo cdInfo;
  CfgType temp = ChooseAlteredCfg<CfgType>(_env, _stats, _dm, _c1, _c2);
  CfgType n = temp;
  if(n == CfgType())
    return false;

  bool isValid = vcm->IsValid(n, _env, _stats, cdInfo, &Callee);
  _cdCounter++;

  if(this->m_debug){
    cout<<"N is "<<isValid<<"::"<<n<<endl;
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
    VID nvid = pathGraph.AddVertex(n);
    CfgType c2, c3;
    bool b1 = lpMethod->IsConnected(_env, _stats, _dm, _c1, n, c2, _lpOutput, _positionRes, _orientationRes, true, false, false);
    if(b1){
      pathGraph.AddEdge(svid, nvid, pair<WeightType, WeightType>());
    }
    bool b2 = lpMethod->IsConnected(_env, _stats, _dm, _c2, n, c3, _lpOutput, _positionRes, _orientationRes, true, false, false);
    if(b2){
      pathGraph.AddEdge(gvid, nvid, pair<WeightType, WeightType>());
    }
    if(this->m_debug) {
      VDAddNode(n);
      VDAddTempEdge(_c1, n);
      VDAddTempEdge(_c2, n);
      VDAddTempCfg(c2, false);
      VDAddTempCfg(c3, false);
      cout<<"n-c2::"<<b1<<"::"<<c2<<endl;
      cout<<"n-c3::"<<b2<<"::"<<c3<<endl;
    }
    if(!b1 && c2!=CfgType())
      b1 = ToggleConnect(_env, _stats, _dm, _col, c2, _c1, n, false, _lpOutput, _positionRes, _orientationRes);
    else if(this->m_debug && b1)
      VDAddEdge(_c1, n);
    if(!b2 && c3!=CfgType() && b1)
      b2 = ToggleConnect(_env, _stats, _dm, _col, c3, _c2, n, false, _lpOutput, _positionRes, _orientationRes);
    else if(this->m_debug && b2)
      VDAddEdge(_c2, n);
    if(b1 && b2 && this->m_debug){
      VDQuery(_c1, _c2);
      VDClearAll();
      VDAddTempCfg(_c1, true);
      VDAddTempCfg(_c2, true);
      VDAddTempCfg(_col, false);
      VDAddTempCfg(n, isValid);
    }
    if(this->m_recordKeep) CalcStats(_stats, b1&&b2, true);
    return b1&&b2;
  }
  else{
    bool b = ToggleConnect(_env, _stats, _dm, _col, n, _c1, _c2, false, _lpOutput, _positionRes, _orientationRes);
    if(b && this->m_debug){
      VDQuery(_c1, _c2);
      VDClearAll();
      VDAddTempCfg(_c1, true);
      VDAddTempCfg(_c2, true);
      VDAddTempCfg(_col, false);
      VDAddTempCfg(n, isValid);
    }
    if(this->m_recordKeep) CalcStats(_stats, b, false);
    return b;
  }
};

template<class MPTraits>
void
ToggleLP<MPTraits>::CalcStats(StatClass& _stats, bool _val, bool _toggle){
  if(_val){
    if(_toggle)
      _stats.IncLPStat("Toggle::FreeSuccess", 1);
    else
      _stats.IncLPStat("Toggle::CollisionSuccess", 1);
    _stats.AddToHistory("Toggle::IterationSuccess", m_iterations);
  }
  else{
    if(_toggle)
      _stats.IncLPStat("Toggle::FreeFailure", 1);
    else
      _stats.IncLPStat("Toggle::CollisionFailure", 1);
    _stats.AddToHistory("Toggle::IterationFailure", m_iterations);
    if(m_degeneracyReached){
      _stats.IncLPStat("Toggle::DegenerateFailure", 1);
      _stats.IncLPStat("Toggle::DegenerateFailureIter", m_iterations);
    }
    else{
      _stats.IncLPStat("Toggle::BlockingFailure", 1);
      _stats.IncLPStat("Toggle::BlockingFailureIter", m_iterations);
    }
  }
  _stats.AddToHistory("Toggle::Iteration", m_iterations);
  _stats.IncLPStat("Toggle::TotalIterations", m_iterations);
  if(m_iterations>_stats.GetLPStat("Toggle::MaxIterations")) _stats.SetLPStat("Toggle::MaxIterations", m_iterations);


  double freeSuccess = _stats.GetLPStat("Toggle::FreeSuccess");
  double collisionSuccess = _stats.GetLPStat("Toggle::CollisionSuccess");
  double freeFailure = _stats.GetLPStat("Toggle::FreeFailure");
  double collisionFailure = _stats.GetLPStat("Toggle::CollisionFailure");
  double degenerateFailure = _stats.GetLPStat("Toggle::DegenerateFailure");
  double blockingFailure = _stats.GetLPStat("Toggle::BlockingFailure");
  _stats.SetLPStat("Toggle::FreeSuccess%", freeSuccess/(freeSuccess+freeFailure)); 
  _stats.SetLPStat("Toggle::CollisionSuccess%", collisionSuccess/(collisionSuccess+collisionFailure)); 
  _stats.SetLPStat("Toggle::TotalSuccess%", (freeSuccess+collisionSuccess)/(collisionSuccess+collisionFailure+freeSuccess+freeFailure)); 
  _stats.SetLPStat("Toggle::IterAvg", _stats.GetLPStat("Toggle::TotalIterations")/_stats.GetLPStat("Toggle::TotalCalls")); 
  _stats.SetLPStat("Toggle::DegeneracyFailure%", degenerateFailure/(collisionFailure+freeFailure)); 
  _stats.SetLPStat("Toggle::BlockingFailure%", blockingFailure/(collisionFailure+freeFailure)); 
  _stats.SetLPStat("Toggle::DegenerateIterAvg", _stats.GetLPStat("Toggle::DegenerateFailureIter")/degenerateFailure); 
  _stats.SetLPStat("Toggle::BlockingIterAvg", _stats.GetLPStat("Toggle::BlockingFailureIter")/blockingFailure); 
}

template<class MPTraits>
bool
ToggleLP<MPTraits>::ToggleConnect(Environment* _env, StatClass& _stats, DistanceMetricPointer _dm, 
    const CfgType& _s, const CfgType& _g, const CfgType& _n1, const CfgType& _n2, bool _toggle,
    LPOutput<MPTraits>* _lpOutput, double _positionRes, double _orientationRes, int _depth) {

  if(_depth>m_maxIter) return false;

  ++m_iterations;

  if(this->m_debug) cout<<"ToggleConnect::"<<_toggle<<"\n\ts::"<<_s<<"\n\tg::"<<_g<<"\n\tn1::"<<_n1<<"\n\tn2::"<<_n2<<endl;

  //set up variables for VC and LP
  LocalPlannerPointer lpMethod = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);

  if(this->m_debug) VDAddTempEdge(_s, _g);

  //check connection between source and goal
  CfgType c; // collision CfgType
  if(!_toggle) this->GetMPProblem()->GetValidityChecker(m_vcLabel)->ToggleValidity();
  bool connect = lpMethod->IsConnected(_env, _stats, _dm, _s, _g, c, _lpOutput, _positionRes, _orientationRes, true, false, false);
  if(!_toggle) this->GetMPProblem()->GetValidityChecker(m_vcLabel)->ToggleValidity();

  if(this->m_debug) cout<<"connect::"<<connect<<"\n\tc::"<<c<<endl;

  //c is outside of the bounding box, degenerate case
  if(!connect && c == CfgType()) return false;

  //successful connection, add the edge and return validity state
  if(connect){
    if(_toggle){
      pathGraph.AddEdge(pathGraph.GetVID(_s), pathGraph.GetVID(_g), pair<WeightType, WeightType>());
    }
    if(this->m_debug && _toggle)
      VDAddEdge(_s, _g);
    return _toggle;
  }

  if(!_toggle){
    pathGraph.AddVertex(c);
  }

  if(this->m_debug) VDAddTempCfg(c, !_toggle);
  if(this->m_debug && !_toggle) VDAddNode(c);

  //look for degeneracies
  if(_n1==c || _n2==c){
    m_degeneracyReached=true; 
    return false;
  }
  typedef typename deque<CfgType>::iterator CIT;
  for(CIT cit = m_colHist.begin(); cit!=m_colHist.end(); cit++){
    if(c==*cit){
      m_degeneracyReached=true; 
      return false;
    }
  }
  if(m_colHist.size()>=50) m_colHist.pop_back();
  m_colHist.push_front(c);

  //recurse
  if(!_toggle)
    return ToggleConnect(_env, _stats, _dm, _n1, c, _s, _g, !_toggle, _lpOutput, _positionRes, _orientationRes, _depth+1) &&
      ToggleConnect(_env, _stats, _dm, _n2, c, _s, _g, !_toggle, _lpOutput, _positionRes, _orientationRes, _depth+1);
  else
    return ToggleConnect(_env, _stats, _dm, _n1, c, _s, _g, !_toggle, _lpOutput, _positionRes, _orientationRes, _depth+1) ||
      ToggleConnect(_env, _stats, _dm, _n2, c, _s, _g, !_toggle, _lpOutput, _positionRes, _orientationRes, _depth+1);
} 

template<class MPTraits>
vector<typename MPTraits::CfgType> 
ToggleLP<MPTraits>::ReconstructPath(Environment* _env, DistanceMetricPointer _dm, 
    const CfgType& _c1, const CfgType& _c2, const vector<CfgType>& _intermediates, 
    double _posRes, double _oriRes){
  StatClass dummyStats;
  LocalPlannerPointer lpMethod = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);
  LPOutput<MPTraits>* lpOutput = new LPOutput<MPTraits>();
  LPOutput<MPTraits>* dummyLPOutput = new LPOutput<MPTraits>();
  CfgType col;
  if(_intermediates.size()>0){
    lpMethod->IsConnected(_env, dummyStats, _dm, _c1, _intermediates[0], col, dummyLPOutput, _posRes, _oriRes, false, true, false);
    for(size_t j = 0; j<dummyLPOutput->path.size(); j++)
      lpOutput->path.push_back(dummyLPOutput->path[j]);
    for(size_t i = 0; i<_intermediates.size()-1; i++){
      lpOutput->path.push_back(_intermediates[i]);
      lpMethod->IsConnected(_env, dummyStats, _dm, _intermediates[i], _intermediates[i+1], col, dummyLPOutput, _posRes, _oriRes, false, true, false);
      for(size_t j = 0; j<dummyLPOutput->path.size(); j++)
        lpOutput->path.push_back(dummyLPOutput->path[j]);
    }
    lpOutput->path.push_back(_intermediates[_intermediates.size()-1]);
    lpMethod->IsConnected(_env, dummyStats, _dm, _intermediates[_intermediates.size()-1], _c2, col, dummyLPOutput, _posRes, _oriRes, false, true, false);
    for(size_t j = 0; j<dummyLPOutput->path.size(); j++)
      lpOutput->path.push_back(dummyLPOutput->path[j]);
  }
  else {
    lpMethod->IsConnected(_env, dummyStats, _dm, _c1, _c2, col, dummyLPOutput, _posRes, _oriRes, false, true, false);
    for(size_t j = 0; j<dummyLPOutput->path.size(); j++)
      lpOutput->path.push_back(dummyLPOutput->path[j]);
  }
  vector<CfgType> path = lpOutput->path;
  delete lpOutput;
  delete dummyLPOutput;
  return path;
}

#endif
