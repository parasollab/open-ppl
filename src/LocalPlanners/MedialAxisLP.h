/* MedialAxisLP.h
 * This class defines the medial axis local planner which performs a
 * push of the pathway connecting 2 medial axis configurations along
 * the medial axis.
 */

#ifndef MEDIALAXISLP_H_
#define MEDIALAXISLP_H_

#include "LocalPlannerMethod.h"
#include "StraightLine.h"
#include "ValidityCheckers/MedialAxisClearanceValidity.h"

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
        double _macEpsilon = 0.01, size_t _maxIter = 2);
    MedialAxisLP(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~MedialAxisLP();

    virtual void PrintOptions(ostream& _os) const;

    MedialAxisUtility<MPTraits>& GetMedialAxisUtility() {
      return m_medialAxisUtility;
    }

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = true,
        bool _saveFailedPath = true);

    virtual vector<CfgType> ReconstructPath(
        const CfgType& _c1, const CfgType& _c2,
        const vector<CfgType>& _intermediates,
        double _posRes, double _oriRes);

  private:
    void Init();

    bool IsConnectedRec(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _posRes, double _oriRes, size_t _itr = 0);

    bool EpsilonClosePath(
        const CfgType& _c1, const CfgType& _c2, CfgType& _mid,
        LPOutput<MPTraits>* _lpOutput,
        double _posRes, double _oriRes);

    bool IsConnectedIter(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _posRes, double _oriRes);

    bool IsConnectedBin(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _posRes, double _oriRes);

    void RemoveBranches(LPOutput<MPTraits>* _lpOutput);

    void ReduceNoise(const CfgType& _c1, const CfgType& _c2,
        LPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes);

    string m_controller;

    MedialAxisUtility<MPTraits> m_medialAxisUtility; //stores operations and
    //variables for medial axis
    double m_macEpsilon; //some epsilon
    size_t m_maxIter; //maximum depth of recursion
    double m_r; //factor of the resolution

    MedialAxisClearanceValidity<MPTraits> m_macVCM;  //mac validity checker
    StraightLine<MPTraits> m_envLP, m_macLP;     //straight line local planners

    bool m_macVCAdded;

  private:
    string m_dmLabel;
};

// Definitions for Constructors and Destructor
template<class MPTraits>
MedialAxisLP<MPTraits>::MedialAxisLP(
    MedialAxisUtility<MPTraits> _medialAxisUtility,
    double _macEpsilon, size_t _maxIter) :
  LocalPlannerMethod<MPTraits>(),
  m_medialAxisUtility(_medialAxisUtility),
  m_macEpsilon(_macEpsilon), m_maxIter(_maxIter) {
    this->SetMPProblem(_medialAxisUtility.GetMPProblem());
    Init();
  }

template<class MPTraits>
MedialAxisLP<MPTraits>::MedialAxisLP(MPProblemType* _problem, XMLNodeReader& _node) :
  LocalPlannerMethod<MPTraits>(_problem, _node), m_medialAxisUtility(_problem, _node) {
    m_medialAxisUtility.SetDebug(false);
    m_controller = _node.stringXMLParameter("controller", true, "", "Which algorithm to run?");
    transform(m_controller.begin(), m_controller.end(), m_controller.begin(), ::toupper);
    m_maxIter = _node.numberXMLParameter("maxIter", true, 2, 1, MAX_INT, "Maximum Number of Iterations");
    if(m_controller == "RECURSIVE")
      m_macEpsilon = _node.numberXMLParameter("macEpsilon", true, 0.1, 0.0, MAX_DBL, "Epsilon-Close to the MA");
    else if(m_controller == "ITERATIVE" || m_controller == "BINARY")
      m_r = _node.numberXMLParameter("r", true, 1.0, 0.0, MAX_DBL, "Resolution for Iter and Bin");
    else {
      cerr << "ERROR::MALP::Unknown controller '" << m_controller << "'. Exiting." << endl;
      exit(1);
    }
    _node.warnUnrequestedAttributes();
    Init();
  }

template<class MPTraits>
MedialAxisLP<MPTraits>::~MedialAxisLP() {}

template<class MPTraits>
void
MedialAxisLP<MPTraits>::Init() {
  this->SetName("MedialAxisLP");

  //Construct a medial axis clearance validity
  m_macVCAdded = false;
  m_macVCM = MedialAxisClearanceValidity<MPTraits>(m_medialAxisUtility, m_macEpsilon);

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
MedialAxisLP<MPTraits>::PrintOptions(ostream& _os) const {
  LocalPlannerMethod<MPTraits>::PrintOptions(_os);
  m_medialAxisUtility.PrintOptions(_os);
  _os << "\tcdLPMethod = " << m_envLP.GetNameAndLabel() << endl
    << "\tmacLPMethod = " << m_macLP.GetNameAndLabel() << endl
    << "\tmacVCMethod = " << m_macVCM.GetNameAndLabel() <<endl
    << "\tmacEpsilon = " << m_macEpsilon << endl
    << "\tmaxIter = " << m_maxIter << endl;
}

// Main IsConnected Function
template<class MPTraits>
bool
MedialAxisLP<MPTraits>::IsConnected(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) {
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  if(!m_macVCAdded) {
    m_macVCAdded = true;
    this->GetMPProblem()->AddValidityChecker(
        ValidityCheckerPointer(&m_macVCM),
        "MAC::" + this->GetNameAndLabel());
  }

  //Check that appropriate save path variables are set
  if(!_checkCollision) {
    if(this->m_debug) {
      cerr << "WARNING, in MedialAxisLP::IsConnected: checkCollision should be "
        << "set to true (false is not applicable for this local planner), "
        << "setting to true.\n";
    }
    _checkCollision = true;
  }
  if(!_savePath) {
    if(this->m_debug) {
      cerr << "WARNING, in MedialAxisLP::IsConnected: savePath should be set "
        << "to true (false is not applicable for this local planner), "
        << "setting to true.\n";
    }
    _savePath = true;
  }
  if(!_saveFailedPath) {
    if(this->m_debug) {
      cerr << "WARNING, in MedialAxisLP::IsConnected: saveFailedPath should be "
        << "set to true (false is not applicable for this local planner), "
        << "setting to true.\n";
    }
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
  stats->IncLPAttempts(this->GetNameAndLabel());
  if(m_controller == "RECURSIVE") {
    if(this->m_debug) {
      VDComment("Initial CFGs");
      VDAddTempCfg(_c1,true);
      VDAddTempCfg(_c2,true);
      VDClearComments();
    }

    connected = IsConnectedRec(_c1, _c2, _col, _lpOutput, _positionRes, _orientationRes, 1);

    if(this->m_debug) {
      VDClearLastTemp();
      VDClearLastTemp();
    }

  }
  else if(m_controller == "ITERATIVE") {
    connected = IsConnectedIter(_c1, _c2, _col, _lpOutput, _positionRes, _orientationRes);
  }
  else if(m_controller == "BINARY") {
    connected = IsConnectedBin(_c1, _c2, _col, _lpOutput, _positionRes, _orientationRes);
  }
  else {
    cerr << "ERROR::MALP::Unknown controller '" << m_controller << "'. Exiting." << endl;
    exit(1);
  }

  if(connected) {
    //first post-process solution
    if(!m_medialAxisUtility.GetExactClearance())
      ReduceNoise(_c1, _c2, _lpOutput, _positionRes, _orientationRes);
    RemoveBranches(_lpOutput);

    //second increment stats and save edge
    stats->IncLPConnections(this->GetNameAndLabel() );
    _lpOutput->m_edge.first.SetWeight(_lpOutput->m_path.size());
    _lpOutput->m_edge.second.SetWeight(_lpOutput->m_path.size());
    _lpOutput->AddIntermediatesToWeights(this->m_saveIntermediates);
    _lpOutput->SetLPLabel(this->GetLabel());
  }

  return connected;
}

// Recursive IsConnected function
template<class MPTraits>
bool
MedialAxisLP<MPTraits>::IsConnectedRec(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _posRes, double _oriRes, size_t _itr) {
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  if(this->m_debug) {
    cout << "  MedialAxisLP::IsConnectedRec" << endl
      << "  Start  : " << _c1 << endl
      << "  End    : " << _c2 << endl;
  }

  if(_itr > m_maxIter) return false;

  LPOutput<MPTraits> maLPOutput, tmpLPOutput;
  int cdCounter = 0, nTicks = 0;
  CfgType mid;
  CfgType diff = _c1 - _c2;

  nTicks = (int)max(diff.PositionMagnitude() / _posRes, diff.OrientationMagnitude() / _oriRes);

  if(nTicks <= 1) {
    if(m_envLP.IsConnected(_c1, _c2, _col, &tmpLPOutput,
          _posRes, _oriRes, true, true, true)) {
      for(size_t j = 0; j < tmpLPOutput.m_path.size(); ++j)
        _lpOutput->m_path.push_back(tmpLPOutput.m_path[j]);
      return true;
    }
    else
      return false;
  }

  // Test for MA closeness
  if(EpsilonClosePath(_c1, _c2, mid, &maLPOutput, _posRes, _oriRes)) {
    for(size_t j = 0; j < maLPOutput.m_path.size(); ++j)
      _lpOutput->m_path.push_back(maLPOutput.m_path[j]);
    return true;
  }

  // Failed epsilon close, recurse
  if(this->m_debug) cout << "  New Mid: " << mid << endl;
  if(this->m_debug) VDAddTempCfg(mid, true);
  LPOutput<MPTraits> lpOutputS, lpOutputE;

  if(!IsConnectedRec(_c1, mid, _col, &lpOutputS, _posRes, _oriRes, _itr + 1)) {
    if (this->m_debug) VDClearLastTemp();
    return false;
  }
  if(!IsConnectedRec(mid, _c2, _col, &lpOutputE, _posRes, _oriRes, _itr + 1)) {
    if (this->m_debug) VDClearLastTemp();
    return false;
  }
  for(size_t i = 0; i < lpOutputS.m_intermediates.size(); i++){
    _lpOutput->m_intermediates.push_back(lpOutputS.m_intermediates[i]);
  }
  _lpOutput->m_intermediates.push_back(mid);
  for(size_t i = 0; i < lpOutputE.m_intermediates.size(); i++){
    _lpOutput->m_intermediates.push_back(lpOutputE.m_intermediates[i]);
  }

  // Push path onto local planner output
  _lpOutput->m_path.push_back(_c1);
  for(size_t i = 0; i < lpOutputS.m_path.size(); ++i)
    _lpOutput->m_path.push_back(lpOutputS.m_path[i]);
  _lpOutput->m_path.push_back(mid);
  for(size_t i = 0; i < lpOutputE.m_path.size(); ++i)
    _lpOutput->m_path.push_back(lpOutputE.m_path[i]);
  _lpOutput->m_path.push_back(_c2);

  VDClearLastTemp();
  stats->IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  return true;
};

template<class MPTraits>
bool
MedialAxisLP<MPTraits>::EpsilonClosePath(
    const CfgType& _c1, const CfgType& _c2, CfgType& _mid,
    LPOutput<MPTraits>* _lpOutput,
    double _posRes, double _oriRes) {
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);

  if(this->m_debug) cout << "MedialAxisLP::EpsilonClosePath()" << endl;

  LPOutput<MPTraits> maLPOutput, tmpLPOutput, testLPOutput;
  CfgType col, tmp;
  int nTicks;
  bool passed = true, found = false;

  // Closer than epsilon so calc pushed mid and test at env res
  if(dm->Distance(_c1, _c2) < m_medialAxisUtility.GetEpsilon()) {
    if(this->m_debug) cout << "Segment is shorter than epsilon..." << endl;

    _mid.FindIncrement( _c1, _c2, &nTicks, _posRes, _oriRes);
    for(size_t i = _mid.PosDOF(); i < _mid.DOF(); i++) {
      if(_mid[i] > 0.5)
        _mid[i] -= 1.0;
    }

    _mid *= ((double)nTicks / 2.0);
    _mid += _c1;

    m_medialAxisUtility.PushToMedialAxis(_mid, env->GetBoundary());
    return m_envLP.IsConnected(_c1, _c2, col, _lpOutput,
        _posRes, _oriRes, true, true, true);
  }

  // Calculate relationship between resolution and m_epsilon
  double rEpsilon = dm->Distance(_c1, _c2) / m_medialAxisUtility.GetEpsilon();

  if(this->m_debug) cout << " Ticks wanted: " << rEpsilon << endl;

  tmp.FindIncrement(_c1, _c2, &nTicks, _posRes, _oriRes);

  rEpsilon = (double)nTicks / rEpsilon;
  double posEps = rEpsilon * _posRes;
  double oriEps = rEpsilon * _oriRes;

  if(this->m_debug) cout << " Ticks calculated: " << nTicks << endl;

  // Check if path is epsilon close, call again without collision checking for
  // LPOuput if failed
  maLPOutput.m_path.push_back(_c1);
  passed = m_macLP.IsConnected(_c1, _c2, col, &testLPOutput,
      posEps, oriEps, true, true, true);
  if(passed) {
    for(size_t j = 0; j < testLPOutput.m_path.size(); ++j)
      maLPOutput.m_path.push_back(testLPOutput.m_path[j]);
  }
  else {
    m_macLP.IsConnected(_c1, _c2, col, &maLPOutput, posEps,
        oriEps, false, true, true);
  }
  maLPOutput.m_path.push_back(_c2);

  // Save pushed mid
  vector<pair<CfgType,CfgType> > history = m_macVCM.GetHistory();

  if(this->m_debug)
    cout << "Finding mid cfg, history size::" << history.size() << endl;
  if(history.size() <= 1) {
    if(history.size() == 1 && !(history[0].first == history[0].second)) {
      // Validity Checker didn't push properly
      if(this->m_debug) cout << "VC didn't push properly" << endl;
      _mid = history[0].second;
    }
    else {
      if(this->m_debug) cout << "VC did push properly" << endl;
      _mid = maLPOutput.m_path[maLPOutput.m_path.size() / 2];
      m_medialAxisUtility.PushToMedialAxis(_mid, env->GetBoundary());
    }
  }
  else {
    if(this->m_debug) cout << "Finding mid from history" << endl;
    for(size_t i = 0; i < history.size(); ++i) {
      if(history[i].first == maLPOutput.m_path[maLPOutput.m_path.size() / 2] ||
          history[i].first == maLPOutput.m_path[maLPOutput.m_path.size() / 2 - 1]) {
        _mid = history[i].second;
        found = true;
        break;
      }
    }
    if(!found) {
      if(this->m_debug) cout << "Mid not found, Pushing onto MA" << endl;
      m_medialAxisUtility.PushToMedialAxis(_mid, env->GetBoundary());
    }
  }
  m_macVCM.ClearHistory();

  // If epsilon close, test at env res
  if(passed) {
    if(this->m_debug) {
      cout << "macLPMethod->IsConnected Passed: maLPOutput.size: "
        << maLPOutput.m_path.size() << endl;
    }
    // Test individual segments
    for(size_t i = 0; i < maLPOutput.m_path.size() - 1; ++i) {
      if(m_envLP.IsConnected(maLPOutput.m_path[i],
            maLPOutput.m_path[i + 1], col, &testLPOutput, _posRes, _oriRes, false,
            true, true)) {
        copy(testLPOutput.m_path.begin(), testLPOutput.m_path.end(), back_inserter(tmpLPOutput.m_path));
        if(i != maLPOutput.m_path.size() - 2)
          tmpLPOutput.m_path.push_back(maLPOutput.m_path[i + 1]);
      }
      else {
        passed = false;
        break;
      }
    }
    // If passed, save path and exit
    if(passed == true) {
      if(this->m_debug)
        cout << "envLP.IsConnected Passed: tmpLPOutput.size: " << tmpLPOutput.m_path.size() << endl;
      copy(tmpLPOutput.m_path.begin(), tmpLPOutput.m_path.end(), back_inserter(_lpOutput->m_path));
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

template<class MPTraits>
bool
MedialAxisLP<MPTraits>::IsConnectedIter(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _posRes, double _oriRes) {

  if(this->m_debug) {
    cout << "  MedialAxisLP::IsConnectedIter" << endl
      << "  Start  : " << _c1 << endl
      << "  End    : " << _c2 << endl;

    VDClearAll();
    VDAddTempCfg(_c1, false);
    VDAddTempCfg(_c2, false);
  }


  CfgType curr = _c1, col;
  LPOutput<MPTraits> lpOutput;
  int nticks;
  size_t iter = 0;
  double r = m_r;
  bool skip = false;

  do {

    CfgType prev = curr;

    //Find tick at resolution
    CfgType tick;
    tick.FindIncrement(curr, _c2, &nticks, r*_posRes, r*_oriRes);
    curr += tick;

    //close enough to goal. Generate last segment of path and quit
    if(curr == _c2) {
      //check for valid connection between previous cfg and final cfg
      //if valid, save path.
      if(!m_envLP.IsConnected(prev, _c2, col, &lpOutput, _posRes, _oriRes, true, true, true)) {
        if(this->m_debug) cout << "Couldn't connect prev: " << prev << " to final: " << _c2 << endl;
        return false;
      }

      copy(lpOutput.m_path.begin(), lpOutput.m_path.end(), back_inserter(_lpOutput->m_path));

      if(this->m_debug) {
        VDAddTempEdge(prev, _c2);
        cout << "Final cfg reached. Connected!" << endl;
      }
      return true;
    }

    if(this->m_debug) cout << "Next::" << curr << endl;

    //Push to medial axis
    if(! m_medialAxisUtility.PushToMedialAxis(curr, this->GetMPProblem()->GetEnvironment()->GetBoundary())) {
      if(this->m_debug) cout << "Push failed. Return false." << endl;
      //return false;
      curr = prev;
      r *= 2;
      skip = true;
      continue;
    }

    r = m_r;
    skip = false;

    if(this->m_debug) {
      cout << "Pushed::" << curr << endl;
      VDAddTempCfg(curr, true);
    }

    //check for pushes to previous configs, if true, return false
    if(curr == prev) {
      if(this->m_debug) cout << "Push to prev location. Returning false." << endl;
      return false;
    }

    //check for valid connection between previous cfg and pushed cfg
    //if valid, save path.
    if(!m_envLP.IsConnected(prev, curr, col, &lpOutput, _posRes, _oriRes, true, true, true)) {
      if(this->m_debug) cout << "Couldn't connect prev: " << prev << " to curr: " << curr << endl;
      return false;
    }

    if(this->m_debug) VDAddTempEdge(prev, curr);

    copy(lpOutput.m_path.begin(), lpOutput.m_path.end(), back_inserter(_lpOutput->m_path));

    if(curr != _c2) {
      _lpOutput->m_path.push_back(curr);
      _lpOutput->m_intermediates.push_back(curr);
    }

  } while(iter++ < m_maxIter);

  if(this->m_debug) cout << "Max iter reached. Not connected." << endl;
  return false;
}


template<class MPTraits>
bool
MedialAxisLP<MPTraits>::IsConnectedBin(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _posRes, double _oriRes) {

  if(this->m_debug) {
    cout << "  MedialAxisLP::IsConnectedBin" << endl
      << "  Start  : " << _c1 << endl
      << "  End    : " << _c2 << endl;

    VDClearAll();
    VDAddTempCfg(_c1, false);
    VDAddTempCfg(_c2, false);
  }

  map<double, pair<CfgType, LPOutput<MPTraits> > > path;
  typedef pair<pair<double, CfgType>, pair<double, CfgType> > Segment;
  queue<Segment> segQueue; //queue stores segments to process
  size_t iter = 0;

  segQueue.push(Segment(make_pair(0, _c1), make_pair(1, _c2)));

  while(!segQueue.empty() && iter++ < m_maxIter) {
    //grab next pair
    Segment seg = segQueue.front();
    segQueue.pop();

    int nTicks;
    CfgType incr;
    incr.FindIncrement(seg.first.second, seg.second.second, &nTicks, m_r*_posRes, m_r*_oriRes);
    //if pair is close enough, test a connection and add to final path
    if(nTicks <= 1) {
      LPOutput<MPTraits> lpOutput;
      CfgType col;
      if(!m_envLP.IsConnected(seg.first.second, seg.second.second, col, &lpOutput, _posRes, _oriRes, true, true, true)) {
        if(this->m_debug) cout << "Connection on segment ({"
          << seg.first.second << "}, {" << seg.second.second << "}) failed." << endl;
        return false;
      }

      //add to path
      path[seg.first.first] = make_pair(seg.second.second, lpOutput);
    }
    //not close enough, so push to MA and push back resulting segments
    else {
      CfgType mid = (seg.first.second + seg.second.second)/2;
      double midval = (seg.first.first + seg.second.first)/2.;

      if(!m_medialAxisUtility.PushToMedialAxis(mid, this->GetMPProblem()->GetEnvironment()->GetBoundary())) {
        if(this->m_debug) cout << "Push failed." << endl;
        return false;
      }

      segQueue.push(Segment(seg.first, make_pair(midval, mid)));
      segQueue.push(Segment(make_pair(midval, mid), seg.second));
    }
  }

  if(segQueue.empty()) {
    if(this->m_debug) cout << "Connected!" << endl;

    //assemble path
    typedef typename map<double, pair<CfgType, LPOutput<MPTraits> > >::iterator PIT;
    for(PIT pit = path.begin(); pit != path.end(); ++pit) {
      copy(pit->second.second.m_path.begin(), pit->second.second.m_path.end(), back_inserter(_lpOutput->m_path));
      if(pit->second.first != _c2) {
        _lpOutput->m_path.push_back(pit->second.first);
        _lpOutput->m_intermediates.push_back(pit->second.first);
      }
    }

    return true;
  }
  else {
    if(this->m_debug) cout << "Max iter reached. Not connected..." << endl;
    return false;
  }
}


template<class MPTraits>
vector<typename MPTraits::CfgType>
MedialAxisLP<MPTraits>::ReconstructPath(
    const CfgType& _c1, const CfgType& _c2,
    const vector<CfgType>& _intermediates,
    double _posRes, double _oriRes) {
  LPOutput<MPTraits>* lpOutput = new LPOutput<MPTraits>();
  LPOutput<MPTraits>* dummyLPOutput = new LPOutput<MPTraits>();
  CfgType col;

  if(_intermediates.size() > 0) {
    m_envLP.IsConnected(_c1, _intermediates[0], col,
        dummyLPOutput, _posRes, _oriRes, false, true, false);
    for(size_t j = 0; j < dummyLPOutput->m_path.size(); j++)
      lpOutput->m_path.push_back(dummyLPOutput->m_path[j]);
    for(size_t i = 0; i < _intermediates.size() - 1; i++){
      lpOutput->m_path.push_back(_intermediates[i]);
      m_envLP.IsConnected(_intermediates[i], _intermediates[i + 1],
          col, dummyLPOutput, _posRes, _oriRes, false, true, false);
      for(size_t j = 0; j < dummyLPOutput->m_path.size(); j++)
        lpOutput->m_path.push_back(dummyLPOutput->m_path[j]);
    }
    lpOutput->m_path.push_back(_intermediates[_intermediates.size() - 1]);
    m_envLP.IsConnected(_intermediates[_intermediates.size() - 1],
        _c2, col, dummyLPOutput, _posRes, _oriRes, false, true, false);
    for(size_t j = 0; j < dummyLPOutput->m_path.size(); j++)
      lpOutput->m_path.push_back(dummyLPOutput->m_path[j]);
  }
  else {
    m_envLP.IsConnected(_c1, _c2, col, dummyLPOutput,
        _posRes, _oriRes, false, true, false);
    for(size_t j = 0; j < dummyLPOutput->m_path.size(); j++)
      lpOutput->m_path.push_back(dummyLPOutput->m_path[j]);
  }
  vector<CfgType> path = lpOutput->m_path;
  delete lpOutput;
  delete dummyLPOutput;
  return path;
}

template<class MPTraits>
void
MedialAxisLP<MPTraits>::RemoveBranches(LPOutput<MPTraits>* _lpOutput) {

  if(_lpOutput->m_path.empty())
    return;

  vector<CfgType> newPath, newIntermediates;

  typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_medialAxisUtility.GetDistanceMetricLabel());

  //RemoveBranches Algorithm
  //_path = {q_1, q_2, ..., q_m}
  //for i = 1 -> m
  //  _newPath = _newPath + {q_i}
  //  j <- m
  //  while(d(q_i, q_j) > resolution
  //    j <- j - 1
  //  i <- j
  //return _newPath

  double res = min(env->GetPositionRes(), env->GetOrientationRes());

  typedef typename vector<CfgType>::iterator CIT;
  for(CIT cit = _lpOutput->m_path.begin(); cit != _lpOutput->m_path.end(); ++cit) {
    newPath.push_back(*cit);

    //if *cit is an intermediate add to intermediates
    typename vector<CfgType>::iterator f = find(_lpOutput->m_intermediates.begin(), _lpOutput->m_intermediates.end(), *cit);
    if(f != _lpOutput->m_intermediates.end())
      newIntermediates.push_back(*cit);

    typedef typename vector<CfgType>::reverse_iterator RCIT;
    RCIT rcit = _lpOutput->m_path.rbegin();
    while(dm->Distance(*cit, *rcit) > res)
      rcit++;

    //when q_i != q_j, push q_j onto the new path to avoid skipping it in the loop
    if(cit != rcit.base()-1) {
      newPath.push_back(*rcit);

      //add rcit as new intermediate
      newIntermediates.push_back(*rcit);
    }

    cit = rcit.base()-1;
  }
  //the loop doesn't push the goal of the path, be sure to do it
  newPath.push_back(_lpOutput->m_path.back());

  //save new path and intermediate information
  _lpOutput->m_path = newPath;
  _lpOutput->m_intermediates = newIntermediates;
}

template<class MPTraits>
void
MedialAxisLP<MPTraits>::
ReduceNoise(const CfgType& _c1, const CfgType& _c2,
    LPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes) {

  if(_lpOutput->m_intermediates.empty())
    return;

  //for each segment on polygonal chain
  //  get midpoint between intermediates along path
  //  test connection between previous and new intermediate
  //  if(success)
  //    add new intermediate
  //  else
  //    add old intermediate

  LPOutput<MPTraits> lpOutput;

  CfgType col;
  CfgType prevMid = (_c1 + _lpOutput->m_intermediates[0])/2;

  vector<CfgType> newIntermediates;

  typedef typename vector<CfgType>::iterator CIT;
  for(CIT cit1 = _lpOutput->m_intermediates.begin(), cit2 = cit1 + 1;
      cit2 != _lpOutput->m_intermediates.end(); ++cit1, ++cit2) {

    CfgType mid = (*cit1 + *cit2)/2;

    if(m_envLP.IsConnected(prevMid, mid, col, &lpOutput, _posRes, _oriRes, true, true, true)) {
      if(newIntermediates.empty() || newIntermediates.back() != prevMid)
        newIntermediates.push_back(prevMid);
      newIntermediates.push_back(mid);
    }
    else
      newIntermediates.push_back(*cit1);

    prevMid = mid;
  }

  //check last goal
  CfgType mid = (_c2 + _lpOutput->m_intermediates.back())/2;

  if(m_envLP.IsConnected(prevMid, mid, col, &lpOutput, _posRes, _oriRes, true, true, true)) {
    if(newIntermediates.empty() || newIntermediates.back() != prevMid)
      newIntermediates.push_back(prevMid);
    newIntermediates.push_back(mid);
  }
  else
    newIntermediates.push_back(_lpOutput->m_intermediates.back());

  //reconstruct new path
  _lpOutput->m_path = ReconstructPath(_c1, _c2, newIntermediates, _posRes, _oriRes);
  _lpOutput->m_intermediates = newIntermediates;
}

#endif
