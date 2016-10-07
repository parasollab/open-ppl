#ifndef STRAIGHT_LINE_H_
#define STRAIGHT_LINE_H_

#include "LocalPlannerMethod.h"

#include <deque>

#include "LPOutput.h"
#include "MPProblem/IsClosedChain.h"
#include <boost/utility/enable_if.hpp>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief Validate straight-line path between two configurations
/// @tparam MPTraits Motion planning universe
///
/// This local planner validates straight line paths which is a direct linear
/// interpolation between two configurations in @cfree.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class StraightLine : public LocalPlannerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    StraightLine(const string& _vcLabel = "", bool _evalation = false,
        bool _saveIntermediates = false);
    StraightLine(MPProblemType* _problem, XMLNode& _node);
    virtual ~StraightLine();

    virtual void Print(ostream& _os) const;

    /**
     * Check if two Cfgs could be connected by straight line.
     */
    //Wrapper function to call appropriate impl IsConnectedFunc based on CfgType
    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false) {
      //clear lpOutput
      _lpOutput->Clear();
      bool connected = IsConnectedFunc<CfgType>(_c1, _c2,
          _col, _lpOutput, _positionRes, _orientationRes, _checkCollision,
          _savePath);
      if(connected)
        _lpOutput->SetLPLabel(this->GetLabel());
      return connected;
    }

    // Default for non closed chains
    template<typename Enable>
    bool IsConnectedFunc(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false,
        typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy = 0);

    // Specialization for closed chains
    template<typename Enable>
    bool IsConnectedFunc(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false,
        typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy = 0);

  protected:
    /**
     * Check if two Cfgs could be connected by straight line.
     * This method implements straight line connection local planner
     * by checking collision of each Cfg along the line.
     * If the is any Cfg causes Robot collides with any obstacle,
     * false will be returned.
     */
    virtual bool IsConnectedSLSequential(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    /**
     * Check if two Cfgs could be connected by straight line
     * This method uses binary search to check clearances of Cfgs between _c1 and
     * _c2. If any Cfg with clearance less than 0.001 was found, false will be returned.
     */
    virtual bool IsConnectedSLBinary(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    string m_vcLabel;
    bool m_binaryEvaluation;
};

template<class MPTraits>
StraightLine<MPTraits>::StraightLine(const string& _vcLabel, bool _evalation,
    bool _saveIntermediates) : LocalPlannerMethod<MPTraits>(_saveIntermediates),
  m_vcLabel(_vcLabel), m_binaryEvaluation(_evalation) {
  this->SetName("StraightLine");
}

template<class MPTraits>
StraightLine<MPTraits>::StraightLine(MPProblemType* _problem, XMLNode& _node) :
    LocalPlannerMethod<MPTraits>(_problem, _node) {
  this->SetName("StraightLine");
  m_binaryEvaluation = _node.Read("binaryEvaluation", false, false,
      "binary evalution along the edge");
  m_vcLabel = _node.Read("vcLabel", true, "",
      "Validity Test Label");
}

template<class MPTraits>
StraightLine<MPTraits>::~StraightLine() { }

template<class MPTraits>
void
StraightLine<MPTraits>::Print(ostream& _os) const {
  LocalPlannerMethod<MPTraits>::Print(_os);
  _os << "\tbinary evaluation = " << (m_binaryEvaluation ? "true" : "false")
      << "\n\tvc label = " << m_vcLabel
      << endl;
}

//// default implementation for non closed chains
template<class MPTraits>
template <typename Enable>
bool
StraightLine<MPTraits>::IsConnectedFunc(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath,
    typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy) {

  StatClass* stats = this->GetMPProblem()->GetStatClass();

  stats->IncLPAttempts(this->GetNameAndLabel());
  int cdCounter = 0;

  bool connected;
  if(m_binaryEvaluation)
    connected = IsConnectedSLBinary(_c1, _c2, _col, _lpOutput,
        cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath);
  else
    connected = IsConnectedSLSequential(_c1, _c2, _col, _lpOutput,
        cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath);
  if(connected)
    stats->IncLPConnections(this->GetNameAndLabel());

  stats->IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  return connected;
}

//// specialized implementation for closed chains
template<class MPTraits>
template <typename Enable>
bool
StraightLine<MPTraits>::IsConnectedFunc(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath,
    typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy) {

  Environment* env = this->GetMPProblem()->GetEnvironment();
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  string callee = this->GetNameAndLabel() + "::IsConnectedSLSequential";

  stats->IncLPAttempts(this->GetNameAndLabel());
  int cdCounter = 0;

  bool connected;
  if(CfgType::OrientationsDifferent(_c1, _c2)) {
    CfgType intermediate;
    bool success = intermediate.GetIntermediate(_c1, _c2);
    if(_checkCollision){
      cdCounter++;
      if(!env->InBounds(intermediate) ||
          !vc->IsValid(intermediate, callee)) {
        if(env->InBounds(intermediate))
          _col = intermediate;
        return false;
      }
    }
    if(!success)
      return false;

    if(m_binaryEvaluation) {
      connected = (IsConnectedSLBinary(_c1, intermediate, _col, _lpOutput,
            cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath)
          &&
          IsConnectedSLBinary(intermediate, _c2, _col, _lpOutput,
            cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath)
          );
    } else {
      connected = (IsConnectedSLSequential(_c1, intermediate, _col, _lpOutput,
            cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath)
          &&
          IsConnectedSLSequential(intermediate, _c2, _col, _lpOutput,
            cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath)
          );
    }
  } else {
    if(m_binaryEvaluation) {
      connected = IsConnectedSLBinary(_c1, _c2, _col, _lpOutput,
          cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath);
    } else {
      connected = IsConnectedSLSequential(_c1, _c2, _col, _lpOutput,
          cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath);
    }
  }
  if(connected)
    stats->IncLPConnections(this->GetNameAndLabel());

  stats->IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  return connected;
}

template<class MPTraits>
bool
StraightLine<MPTraits>::IsConnectedSLSequential(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {

  Environment* env = this->GetMPProblem()->GetEnvironment();
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);

  int nTicks;
  CfgType tick;
  tick = _c1;
  CfgType incr;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes, env->GetRdRes());
#else
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);
#endif
  string callee = this->GetNameAndLabel() + "::IsConnectedSLSequential";

  int nIter = 0;
  for(int i = 1; i < nTicks; i++){ //don't need to check the ends, _c1 and _c2
    tick += incr;
    _cdCounter++;
    if(_checkCollision){
      if(!env->InBounds(tick) || !vc->IsValid(tick, callee)) {
        if(env->InBounds(tick))
          _col = tick;
        CfgType negIncr = -incr;
        tick += negIncr;
        _lpOutput->m_edge.first.SetWeight(_lpOutput->m_edge.first.GetWeight() + nIter);
        _lpOutput->m_edge.second.SetWeight(_lpOutput->m_edge.second.GetWeight() + nIter);
        return false;
      }
    }
    if(_savePath) {
      _lpOutput->m_path.push_back(tick);
    }
    nIter++;
  }
  _lpOutput->m_edge.first.SetWeight(_lpOutput->m_edge.first.GetWeight() + nIter);
  _lpOutput->m_edge.second.SetWeight(_lpOutput->m_edge.second.GetWeight() + nIter);

  return true;
};

template<class MPTraits>
bool
StraightLine<MPTraits>::IsConnectedSLBinary(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {

  Environment* env = this->GetMPProblem()->GetEnvironment();
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);

  if(!_checkCollision)
    return IsConnectedSLSequential(_c1, _c2, _col, _lpOutput,
        _cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath);

  string callee = this->GetNameAndLabel() + "::IsConnectedSLBinary";

  int nTicks;
  CfgType incr;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes, env->GetRdRes());
#else
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);
#endif

  deque<pair<int,int> > Q;

  //only perform binary evaluation when the nodes are further apart than the
  //resolution
  if(nTicks > 1)
    Q.push_back(make_pair(0, nTicks));

  while(!Q.empty()) {
    pair<int,int> p = Q.front();
    int i = p.first;
    int j = p.second;
    Q.pop_front();

    int mid = i + (j - i) / 2;

    CfgType midCfg = incr * mid + _c1;

    _cdCounter++;

    if(!env->InBounds(midCfg) || !vc->IsValid(midCfg, callee) ) {
      if(env->InBounds(midCfg))
        _col = midCfg;
      return false;
    }
    else {
      if(i + 1 != mid)
        Q.push_back(make_pair(i, mid));
      if(mid + 1 != j)
        Q.push_back(make_pair(mid, j));
    }
  }

  if(_savePath) {
    CfgType tick = _c1;
    for(int n = 1; n < nTicks; ++n) {
      tick += incr;
      _lpOutput->m_path.push_back(tick);
    }
  }
  _lpOutput->m_edge.first.SetWeight(_lpOutput->m_edge.first.GetWeight() + nTicks - 1);
  _lpOutput->m_edge.second.SetWeight(_lpOutput->m_edge.second.GetWeight() + nTicks - 1);

  return true;
}

#endif
