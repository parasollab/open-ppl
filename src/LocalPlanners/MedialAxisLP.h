/* MedialAxisLP.h
 * This class defines the medial axis local planner which performs a
 * push of the pathway connecting 2 medial axis configurations along
 * the medial axis.
 */

#ifndef MEDIALAXISLP_H_
#define MEDIALAXISLP_H_

#include "LocalPlannerMethod.h"
#include "StraightLine.h"

template<class MPTraits>
class MedialAxisLP : public LocalPlannerMethod<MPTraits> {
  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;

    MedialAxisLP(MedialAxisUtility<MPTraits> _medialAxisUtility = MedialAxisUtility<MPTraits>(),
        double _macEpsilon = 0.01,
        int _maxIter = 2);
    MedialAxisLP(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~MedialAxisLP();

    virtual void PrintOptions(ostream& _os);

    virtual bool IsConnected(Environment* _env, StatClass& _stats,
        DistanceMetricPointer _dm, const CfgType& _c1, const CfgType& _c2, 
        CfgType& _col, LPOutput<MPTraits>* _lpOutput, double _positionRes, 
        double _orientationRes, bool _checkCollision=true, 
        bool _savePath=true, bool _saveFailedPath=true);

    virtual vector<CfgType> ReconstructPath(Environment* _env, DistanceMetricPointer _dm, 
        const CfgType& _c1, const CfgType& _c2, 
        const vector<CfgType>& _intermediates, 
        double _posRes, double _oriRes);

  protected:
    void Init();

    bool IsConnectedRec(Environment* _env, StatClass& _stats,
        DistanceMetricPointer _dm, const CfgType& _c1, const CfgType& _c2, 
        CfgType& _col, LPOutput<MPTraits>* _lpOutput,
        double _posRes, double _oriRes, int _itr=0);

    bool EpsilonClosePath(Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2,
        CfgType& _mid, LPOutput<MPTraits>* _lpOutput, 
        double _posRes, double _oriRes);

    MedialAxisUtility<MPTraits> m_medialAxisUtility; //stores operations and variables for medial axis 
    double m_macEpsilon; //some epsilon
    int m_maxIter; //maximum depth of recursion
    
    MedialAxisClearanceValidity<MPTraits> m_macVCM;  //mac validity checker
    StraightLine<MPTraits> m_envLP, m_macLP;         //straight line local planners
};

// Definitions for Constructors and Destructor
template<class MPTraits>
MedialAxisLP<MPTraits>::MedialAxisLP(MedialAxisUtility<MPTraits> _medialAxisUtility,
    double _macEpsilon, int _maxIter) : 
  LocalPlannerMethod<MPTraits>(), 
  m_medialAxisUtility(_medialAxisUtility), 
  m_macEpsilon(_macEpsilon), m_maxIter(_maxIter) {
    Init();
  }

template<class MPTraits> 
MedialAxisLP<MPTraits>::MedialAxisLP(MPProblemType* _problem, XMLNodeReader& _node) : 
  LocalPlannerMethod<MPTraits>(_problem, _node), m_medialAxisUtility(_problem, _node) {
    m_macEpsilon = _node.numberXMLParameter("macEpsilon", false, 0.1, 0.0, 1.0, "Epsilon-Close to the MA");
    m_maxIter = _node.numberXMLParameter("maxIter", false, 2, 1, 1000, "Maximum Number of Recursive Iterations");
    _node.warnUnrequestedAttributes();
    Init();
  }

template<class MPTraits> 
MedialAxisLP<MPTraits>::~MedialAxisLP() { }

template<class MPTraits>
void
MedialAxisLP<MPTraits>::Init(){
  this->SetName("MedialAxisLP");

  //Construct a medial axis clearance validity
  m_macVCM = MedialAxisClearanceValidity<MPTraits>(m_medialAxisUtility, m_macEpsilon);
  this->GetMPProblem()->AddValidityChecker(ValidityCheckerPointer(&m_macVCM), "MAC::" + this->GetNameAndLabel());

  //Local planner methods
  m_envLP = StraightLine<MPTraits>(m_medialAxisUtility.GetValidityCheckerLabel(), true);
  m_macLP = StraightLine<MPTraits>("MAC::" + this->GetNameAndLabel(), true);    

  //make sure MPProblems point to the correct place
  m_macVCM.SetMPProblem(this->GetMPProblem());
  m_envLP.SetMPProblem(this->GetMPProblem());
  m_macLP.SetMPProblem(this->GetMPProblem());
}

template<class MPTraits> 
void 
MedialAxisLP<MPTraits>::PrintOptions(ostream& _os) {
  LocalPlannerMethod<MPTraits>::PrintOptions(_os);
  m_medialAxisUtility.PrintOptions(_os);
  _os << "\tcdLPMethod = " << m_envLP.GetName() << endl;
  _os << "\tmacLPMethod = " << m_macLP.GetName() << endl;
  _os << "\tmacVCMethod = " << m_macVCM.GetName() <<endl;
  _os << "\tmacEpsilon = " << m_macEpsilon << endl;
  _os << "\tmaxIter = " << m_maxIter << endl;
}

// Main IsConnected Function
template<class MPTraits> 
bool 
MedialAxisLP<MPTraits>::IsConnected(Environment* _env,
    StatClass& _stats, DistanceMetricPointer _dm,
    const CfgType& _c1, const CfgType& _c2, 
    CfgType& _col, LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) {  

  //Check that appropriate save path variables are set
  if(!_checkCollision) {
    if(this->m_debug) cerr << "WARNING, in MedialAxisLP::IsConnected: checkCollision should be set to true (false is not applicable for this local planner), setting to true.\n";
    _checkCollision = true;
  }
  if(!_savePath) {
    if(this->m_debug) cerr << "WARNING, in MedialAxisLP::IsConnected: savePath should be set to true (false is not applicable for this local planner), setting to true.\n";
    _savePath = true;
  }
  if(!_saveFailedPath) {
    if(this->m_debug) cerr << "WARNING, in MedialAxisLP::IsConnected: saveFailedPath should be set to true (false is not applicable for this local planner), setting to true.\n";
    _saveFailedPath = true;
  }

  //Clear lpOutput
  _lpOutput->Clear();

  if(this->m_debug) {
    cout << "\nMedialAxisLP::IsConnected" << endl
      << "Start: " << _c1 << endl
      << "End  : " << _c2 << endl;
  }

  bool connected = false;
  _stats.IncLPAttempts(this->GetNameAndLabel());
  if(this->m_debug) {
    VDComment("Initial CFGs");
    VDAddTempCfg(_c1,true);
    VDAddTempCfg(_c2,true);
    VDClearComments();
  }

  connected = IsConnectedRec(_env, _stats, _dm, _c1, _c2, _col, _lpOutput, _positionRes, _orientationRes, 1);

  if(connected) {
    _stats.IncLPConnections(this->GetNameAndLabel() );  
    _lpOutput->edge.first.SetWeight(_lpOutput->path.size());
    _lpOutput->edge.second.SetWeight(_lpOutput->path.size());
    _lpOutput->AddIntermediatesToWeights(this->m_saveIntermediates);
    _lpOutput->SetLPLabel(this->GetLabel());
  }

  if(this->m_debug) {
    VDClearLastTemp();
    VDClearLastTemp();
  }

  return connected;
}

// Recursive IsConnected function
template<class MPTraits> 
bool 
MedialAxisLP<MPTraits>::IsConnectedRec(Environment* _env, 
    StatClass& _stats, DistanceMetricPointer _dm, 
    const CfgType& _c1, const CfgType& _c2, 
    CfgType& _col, LPOutput<MPTraits>* _lpOutput,
    double _posRes, double _oriRes, int _itr) {

  if(this->m_debug) {
    cout << "  MedialAxisLP::IsConnectedRec" << endl
      << "  Start  : " << _c1 << endl
      << "  End    : " << _c2 << endl;
  }

  if(_itr > m_maxIter) return false;

  LPOutput<MPTraits> maLPOutput, tmpLPOutput;
  int cdCounter=0, nTicks=0;
  CfgType mid;
  CfgType diff = _c1 - _c2;

  nTicks = (int)max(diff.PositionMagnitude()/_posRes,
      diff.OrientationMagnitude()/_oriRes);

  if(nTicks <= 1) {
    if(m_envLP.IsConnected(_env, _stats, _dm, _c1, _c2, _col, &tmpLPOutput, _posRes, _oriRes, true, true, true)) {
      for(size_t j=0; j<tmpLPOutput.path.size(); ++j)
        _lpOutput->path.push_back(tmpLPOutput.path[j]);
      return true;  
    } 
    else {
      return false;
    }
  }

  // Test for MA closeness
  if(EpsilonClosePath(_env, _stats, _dm, _c1, _c2, mid, &maLPOutput, _posRes, _oriRes) ) {
    for(size_t j=0; j<maLPOutput.path.size(); ++j)
      _lpOutput->path.push_back(maLPOutput.path[j]);
    return true;
  }

  // Failed epsilon close, recurse
  if(this->m_debug) cout << "  New Mid: " << mid << endl;
  if(this->m_debug) VDAddTempCfg( mid, true);
  LPOutput<MPTraits> lpOutputS, lpOutputE;

  if(!IsConnectedRec(_env, _stats, _dm, _c1, mid, _col, &lpOutputS, _posRes, _oriRes, _itr+1)) {
    if (this->m_debug) VDClearLastTemp();
    return false;
  }
  if(!IsConnectedRec(_env, _stats, _dm, mid, _c2, _col, &lpOutputE, _posRes, _oriRes, _itr+1)) {
    if (this->m_debug) VDClearLastTemp();
    return false;
  }
  for(size_t i=0; i<lpOutputS.intermediates.size(); i++){
    _lpOutput->intermediates.push_back(lpOutputS.intermediates[i]);
  }
  _lpOutput->intermediates.push_back(mid);
  for(size_t i=0; i<lpOutputE.intermediates.size(); i++){
    _lpOutput->intermediates.push_back(lpOutputE.intermediates[i]);
  }

  // Push path onto local planner output
  _lpOutput->path.push_back(_c1);
  for(size_t i=0; i<lpOutputS.path.size(); ++i)
    _lpOutput->path.push_back(lpOutputS.path[i]);
  _lpOutput->path.push_back(mid);
  for(size_t i=0; i<lpOutputE.path.size(); ++i)
    _lpOutput->path.push_back(lpOutputE.path[i]);
  _lpOutput->path.push_back(_c2);

  VDClearLastTemp();
  _stats.IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);  
  return true;
};

template<class MPTraits> 
bool 
MedialAxisLP<MPTraits>::EpsilonClosePath( Environment *_env,
    StatClass& _stats, DistanceMetricPointer _dm, 
    const CfgType &_c1, const CfgType &_c2, 
    CfgType &_mid, LPOutput<MPTraits>* _lpOutput,
    double _posRes, double _oriRes) {

  if(this->m_debug) cout << "MedialAxisLP::EpsilonClosePath()" << endl;

  LPOutput<MPTraits> maLPOutput, tmpLPOutput, testLPOutput;
  CfgType col, tmp;
  int nTicks;
  bool passed=true, found=false;

  // Closer than epsilon so calc pushed mid and test at _env res
  if(_dm->Distance(_env, _c1, _c2) < m_medialAxisUtility.GetEpsilon()) {
    if(this->m_debug) cout << "Segment is shorter than epsilon..." << endl;

    _mid.FindIncrement( _c1, _c2, &nTicks, _posRes, _oriRes);
    for(size_t i=_mid.PosDOF(); i<_mid.DOF(); i++) {
      if( _mid[i] > 0.5 )
        _mid[i] -= 1.0; 
    }

    _mid *= ((double)nTicks/2.0);
    _mid += _c1;

    m_medialAxisUtility.PushToMedialAxis(_mid, _env->GetBoundary());
    return m_envLP.IsConnected(_env, _stats, _dm, _c1, _c2, col, _lpOutput, _posRes, _oriRes, true, true, true);
  }

  // Calculate relationship between resolution and m_epsilon
  double rEpsilon = _dm->Distance(_env, _c1, _c2)/m_medialAxisUtility.GetEpsilon();

  if(this->m_debug) cout << " Ticks wanted: " << rEpsilon << endl;

  tmp.FindIncrement(_c1, _c2, &nTicks, _posRes, _oriRes);

  rEpsilon = (double)nTicks/rEpsilon;
  double posEps = rEpsilon*_posRes;
  double oriEps = rEpsilon*_oriRes;

  if(this->m_debug) cout << " Ticks calculated: " << nTicks << endl;

  // Check if path is epsilon close, call again without collision checking for LPOuput if failed
  maLPOutput.path.push_back(_c1);
  passed = m_macLP.IsConnected(_env, _stats, _dm, _c1, _c2, col, &testLPOutput, posEps, oriEps, true, true, true);
  if(passed) {
    for(size_t j=0; j<testLPOutput.path.size(); ++j)
      maLPOutput.path.push_back(testLPOutput.path[j]);
  } 
  else {
    m_macLP.IsConnected(_env, _stats, _dm, _c1, _c2, col, &maLPOutput, posEps, oriEps, false, true, true);
  }
  maLPOutput.path.push_back(_c2);

  // Save pushed mid
  vector< pair<CfgType,CfgType> > history = m_macVCM.GetHistory();

  if(this->m_debug) cout << "Finding mid cfg, history size::" << history.size() << endl;
  if(history.size() <= 1) {
    if(history.size() == 1 && !(history[0].first == history[0].second) ) { // Validity Checker didn't push properly
      if(this->m_debug) cout << "VC didn't push properly" << endl;
      _mid = history[0].second;
    } 
    else {
      if(this->m_debug) cout << "VC did push properly" << endl;
      _mid = maLPOutput.path[maLPOutput.path.size()/2];
      m_medialAxisUtility.PushToMedialAxis(_mid, _env->GetBoundary());
    }
  } 
  else {
    if(this->m_debug) cout << "Finding mid from history" << endl;
    for(size_t i=0; i<history.size(); ++i) {
      if(history[i].first == maLPOutput.path[maLPOutput.path.size()/2] ||
          history[i].first == maLPOutput.path[maLPOutput.path.size()/2-1]) {
        _mid = history[i].second;
        found = true;
        break;
      }
    }
    if(!found) {
      if(this->m_debug) cout << "Mid not found, Pushing onto MA" << endl;
      m_medialAxisUtility.PushToMedialAxis(_mid, _env->GetBoundary());
    }
  }
  m_macVCM.ClearHistory();

  // If epsilon close, test at _env res
  if(passed) {
    if(this->m_debug) cout << "macLPMethod->IsConnected Passed: maLPOutput.size: " << maLPOutput.path.size() << endl;
    // Test individual segments
    for(size_t i=0; i<maLPOutput.path.size()-1; ++i) {
      if(m_envLP.IsConnected(_env, _stats, _dm,maLPOutput.path[i], maLPOutput.path[i+1], col,
            &testLPOutput, _posRes, _oriRes, false, true, true)) {
        for(size_t j=0; j<testLPOutput.path.size(); ++j)
          tmpLPOutput.path.push_back(testLPOutput.path[j]);
        if(i != maLPOutput.path.size()-2)
          tmpLPOutput.path.push_back(maLPOutput.path[i+1]);
      } 
      else {
        passed = false;
        break;
      }
    }
    // If passed, save path and exit
    if(passed == true) {
      if(this->m_debug) cout << "envLP.IsConnected Passed: tmpLPOutput.size: " << tmpLPOutput.path.size() << endl;
      for (size_t j=0; j<tmpLPOutput.path.size(); ++j)
        _lpOutput->path.push_back(tmpLPOutput.path[j]);
      return true;
    } 
    else {
      return false;
    }
  } 
  else
    return false;

}

template<class MPTraits>
vector<typename MPTraits::CfgType> 
MedialAxisLP<MPTraits>::ReconstructPath(Environment* _env,
    DistanceMetricPointer _dm, const CfgType& _c1,
    const CfgType& _c2, const vector<CfgType>& _intermediates,
    double _posRes, double _oriRes){
  StatClass dummyStats;
  LPOutput<MPTraits>* lpOutput = new LPOutput<MPTraits>();
  LPOutput<MPTraits>* dummyLPOutput = new LPOutput<MPTraits>();
  CfgType col;

  if(_intermediates.size() > 0){
    m_envLP.IsConnected(_env, dummyStats, _dm, _c1, _intermediates[0], col, dummyLPOutput, _posRes, _oriRes, false, true, false);
    for(size_t j = 0; j<dummyLPOutput->path.size(); j++)
      lpOutput->path.push_back(dummyLPOutput->path[j]);
    for(size_t i = 0; i<_intermediates.size()-1; i++){
      lpOutput->path.push_back(_intermediates[i]);
      m_envLP.IsConnected(_env, dummyStats, _dm, _intermediates[i], _intermediates[i+1], col, dummyLPOutput, _posRes, _oriRes, false, true, false);
      for(size_t j = 0; j<dummyLPOutput->path.size(); j++)
        lpOutput->path.push_back(dummyLPOutput->path[j]);
    }
    lpOutput->path.push_back(_intermediates[_intermediates.size()-1]);
    m_envLP.IsConnected(_env, dummyStats, _dm, _intermediates[_intermediates.size()-1], _c2, col, dummyLPOutput, _posRes, _oriRes, false, true, false);
    for(size_t j = 0; j<dummyLPOutput->path.size(); j++)
      lpOutput->path.push_back(dummyLPOutput->path[j]);
  }
  else {
    m_envLP.IsConnected(_env, dummyStats, _dm, _c1, _c2, col, dummyLPOutput, _posRes, _oriRes, false, true, false);
    for(size_t j = 0; j<dummyLPOutput->path.size(); j++)
      lpOutput->path.push_back(dummyLPOutput->path[j]);
  }
  vector<CfgType> path = lpOutput->path;
  delete lpOutput;
  delete dummyLPOutput;
  return path;
}

#endif
