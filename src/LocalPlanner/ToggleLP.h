#ifndef Toggle_h
#define Toggle_h

#include "ValidityChecker.hpp"
#include "LocalPlannerMethod.h"
#include "LocalPlanners.h"
#include "MPStrategy.h"

template<class CFG, class WEIGHT>
class LocalPlanners;

template <class CFG, class WEIGHT>
class ToggleLP: public LocalPlannerMethod<CFG, WEIGHT> {
  public:

    ToggleLP(string _vc = "", string _lp = "", int _maxIter = 0);
    ToggleLP(XMLNodeReader& _node, MPProblem* _problem);
    void InitVars();
    virtual ~ToggleLP();

    void CalcStats(StatClass& _stats, bool _var, bool _toggle);
    virtual void PrintOptions(ostream& _os);
    virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

    virtual bool IsConnected(Environment* _env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod> _dm, const CFG& _c1, const CFG& _c2, CFG& _col, 
        LPOutput<CFG, WEIGHT>* _lpOutput, double _positionRes, double _orientationRes,
        bool _checkCollision=true, bool _savePath=false, bool _saveFailedPath=false);

  protected:

    // Default for non closed chains - alters midpoint to a distance delta away on a random ray
    template <typename Enable>
      CFG ChooseAlteredCfg(Environment* _env, StatClass& _stats,
          shared_ptr<DistanceMetricMethod> _dm,
          const CFG& _c1, const CFG& _c2,
          typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy = 0
          );

    // Specialization for closed chains - choose random point
    template <typename Enable>
      CFG ChooseAlteredCfg(Environment* _env, StatClass& _stats,
          shared_ptr<DistanceMetricMethod> _dm,
          const CFG& _c1, const CFG& _c2,
          typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy = 0
          );

    bool IsConnectedToggle(Environment* _env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod> _dm, const CFG& _c1, const CFG& _c2, CFG& _col,
        LPOutput<CFG,WEIGHT>* _lpOutput, int& _cdCounter, double _positionRes, double _orientationRes,
        bool _checkCollision=true, bool _savePath=false, bool _saveFailedPath=false);

    bool ToggleConnect(Environment* _env, StatClass& _stats, 
        shared_ptr<DistanceMetricMethod> _dm, const CFG& _s, const CFG& _g, const CFG& _n1, const CFG& _n2, bool _toggle,
        LPOutput<CFG, WEIGHT>* _lpOutput, double _positionRes, double _orientationRes, int _depth=0);

  private:
    //input
    string m_vc, m_lp;
    int m_maxIter;

    //needed variables for record keeping and cycle detection
    double m_iterations;
    bool m_degeneracyReached;
    deque<CFG> m_colHist;
};

template <class CFG, class WEIGHT>
ToggleLP<CFG, WEIGHT>::ToggleLP(string _vc, string _lp, int _maxIter) : 
  m_vc(_vc), m_lp(_lp), m_maxIter(_maxIter) {
    InitVars();
  }

template <class CFG, class WEIGHT>
ToggleLP<CFG, WEIGHT>::ToggleLP(XMLNodeReader& _node, MPProblem* _problem) : LocalPlannerMethod<CFG, WEIGHT>(_node,_problem) {
  InitVars();
  if(this->m_debug) cout<<this->GetNameAndLabel()<<endl;
  m_vc = _node.stringXMLParameter("vcMethod", true, "", "Validity Test Method");
  m_lp = _node.stringXMLParameter("lpMethod", true, "", "Local Planner Method");
  m_maxIter = _node.numberXMLParameter("maxIter", false, 10, 0, MAX_INT, "Maximum number of m_iterations");

  _node.warnUnrequestedAttributes();

  if(this->m_debug) cout<<"~"<<this->GetNameAndLabel()<<endl;
}

template <class CFG, class WEIGHT>
void 
ToggleLP<CFG, WEIGHT>::InitVars(){
  this->SetName("ToggleLP");
  m_iterations = 0;
}

template <class CFG, class WEIGHT>
ToggleLP<CFG, WEIGHT>::~ToggleLP() {
}

template <class CFG, class WEIGHT>
void
ToggleLP<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << "    " << this->GetName() << "::  ";
  _os << "maxIter =  " << m_maxIter << " ";
  _os << "vc =  " << m_vc << " ";
  _os << "lp =  " << m_lp << " ";
  _os << endl;
}

template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
ToggleLP<CFG, WEIGHT>::CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * copy = new ToggleLP<CFG, WEIGHT>(*this);
  return copy;
}

template <class CFG, class WEIGHT>
bool 
ToggleLP<CFG, WEIGHT>::IsConnected(Environment* _env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod> _dm, const CFG& _c1, const CFG& _c2, CFG& _col, 
    LPOutput<CFG, WEIGHT>* _lpOutput, double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) { 
  //clear lpOutput
  _lpOutput->Clear();

  _stats.IncLPAttempts(this->GetNameAndLabel());
  int cdCounter = 0; 

  bool connected;
  connected = IsConnectedToggle(_env, _stats, _dm, _c1, _c2, _col, 
      _lpOutput, cdCounter, _positionRes, _orientationRes, 
      _checkCollision, _savePath, _saveFailedPath);
  if(connected){
    _stats.IncLPConnections(this->GetNameAndLabel());
    _lpOutput->SetLPLabel(this->GetNameAndLabel());
  }

  _stats.IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  return connected;
}

// Default for non closed chains - alters midpoint to a distance delta away on a random ray
template <class CFG, class WEIGHT>
template <typename Enable>
CFG ToggleLP<CFG, WEIGHT>::ChooseAlteredCfg(Environment* _env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod> _dm, const CFG& _c1, const CFG& _c2,
    typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy){
  size_t attempts = 0;
  CFG mid, temp;
  mid.add(_c1, _c2);
  mid.divide(_c1, 2.0);
  do{
    CFG incr;
    double dist = _dm->Distance(_env, _c1, _c2) * sqrt(2.0)/2.0;
    incr.GetRandomRay(dist, _env, _dm);
    temp.add(incr, mid);
  }while(!temp.InBoundingBox(_env) && attempts++<10);
  if(attempts==10){ 
    _stats.IncLPStat("Toggle::MaxAttemptsForRay", 1);
    return CFG();
  }
  return temp;
}

// Specialization for closed chains - choose random point
template <class CFG, class WEIGHT>
template <typename Enable>
CFG ToggleLP<CFG, WEIGHT>::ChooseAlteredCfg(Environment* _env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod> _dm, 
    const CFG& _c1, const CFG& _c2,
    typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy){
  CFG temp;
  do{
    temp.GetRandomCfg(_env);
  }while(!temp.InBoundingBox(_env));
  return temp;
}

template <class CFG, class WEIGHT>
bool
ToggleLP<CFG, WEIGHT>::IsConnectedToggle(Environment* _env, StatClass& _stats,
    shared_ptr<DistanceMetricMethod> _dm, const CFG& _c1, const CFG& _c2, CFG& _col,
    LPOutput<CFG,WEIGHT>* _lpOutput, int& _cdCounter, double _positionRes, double _orientationRes,
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
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(m_vc);
  typename LocalPlanners<CFG, WEIGHT>::LocalPlannerPointer lpMethod = this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_lp);

  if(lpMethod->IsConnected(_env, _stats, _dm, _c1, _c2, _col, _lpOutput, _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath)){
    return true;
  }
  if(_col == CFG())
    return false;
  if(this->m_debug) cout<<"col::"<<_col<<endl;

  if(this->m_recordKeep) _stats.IncLPStat("Toggle::TotalCalls", 1);

  CDInfo cdInfo;
  CFG temp = ChooseAlteredCfg<CFG>(_env, _stats, _dm, _c1, _c2);
  CFG n = temp;
  if(n == CFG())
    return false;

  bool isValid = vc->IsValid(vcm, n, _env, _stats, cdInfo, true, &Callee);
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
    CFG c2, c3;
    bool b1 = lpMethod->IsConnected(_env, _stats, _dm, _c1, n, c2, _lpOutput, _positionRes, _orientationRes, true, false, false);
    bool b2 = lpMethod->IsConnected(_env, _stats, _dm, _c2, n, c3, _lpOutput, _positionRes, _orientationRes, true, false, false);
    if(this->m_debug) {
      VDAddNode(n);
      VDAddTempEdge(_c1, n);
      VDAddTempEdge(_c2, n);
      VDAddTempCfg(c2, false);
      VDAddTempCfg(c3, false);
      cout<<"n-c2::"<<b1<<"::"<<c2<<endl;
      cout<<"n-c3::"<<b2<<"::"<<c3<<endl;
    }
    if(!b1 && c2!=CFG())
      b1 = ToggleConnect(_env, _stats, _dm, _col, c2, _c1, n, false, _lpOutput, _positionRes, _orientationRes);
    else if(this->m_debug && b1)
      VDAddEdge(_c1, n);
    if(!b2 && c3!=CFG() && b1)
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

template <class CFG, class WEIGHT>
void
ToggleLP<CFG, WEIGHT>::CalcStats(StatClass& _stats, bool _val, bool _toggle){
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

template <class CFG, class WEIGHT>
bool
ToggleLP<CFG, WEIGHT>::ToggleConnect(Environment* _env, StatClass& _stats, shared_ptr<DistanceMetricMethod> _dm, 
    const CFG& _s, const CFG& _g, const CFG& _n1, const CFG& _n2, bool _toggle,
    LPOutput<CFG,WEIGHT>* _lpOutput, double _positionRes, double _orientationRes, int _depth) {

  if(_depth>m_maxIter) return false;

  ++m_iterations;

  if(this->m_debug) cout<<"ToggleConnect::"<<_toggle<<"\n\ts::"<<_s<<"\n\tg::"<<_g<<"\n\tn1::"<<_n1<<"\n\tn2::"<<_n2<<endl;

  //set up variables for VC and LP
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename LocalPlanners<CFG, WEIGHT>::LocalPlannerPointer lpMethod = this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_lp);

  if(this->m_debug) VDAddTempEdge(_s, _g);

  //check connection between source and goal
  CFG c; // collision CFG
  if(!_toggle) vc->ToggleValidity();
  bool connect = lpMethod->IsConnected(_env, _stats, _dm, _s, _g, c, _lpOutput, _positionRes, _orientationRes, true, false, false);
  if(!_toggle) vc->ToggleValidity();

  if(this->m_debug) cout<<"connect::"<<connect<<"\n\tc::"<<c<<endl;

  //c is outside of the bounding box, degenerate case
  if(!connect && c == CFG()) return false;

  //successful conntection, add the edge and return validity state
  if(connect){
    if(this->m_debug && _toggle)
      VDAddEdge(_s, _g);
    return _toggle;
  }

  if(this->m_debug) VDAddTempCfg(c, !_toggle);
  if(this->m_debug && !_toggle) VDAddNode(c);

  //look for degeneracies
  if(_n1==c || _n2==c){
    m_degeneracyReached=true; 
    return false;
  }
  typedef typename deque<CFG>::iterator CIT;
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

#endif
