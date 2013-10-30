/**
 * StraightLine.h
 *
 * This class performs straight line local planning which is used by a variety
 * of computations, including other local planners.
 */

#ifndef STRAIGHTLINE_H_
#define STRAIGHTLINE_H_

#include <deque>
#include "LocalPlannerMethod.h"
#include "LPOutput.h"
#include "MPProblem/IsClosedChain.h"

template <class MPTraits>
class StraightLine : public LocalPlannerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    StraightLine(string _vcLabel = "", bool _evalation = false);
    StraightLine(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~StraightLine();

    virtual void PrintOptions(ostream& _os) const;

    /**
     * Check if two Cfgs could be connected by straight line.
     */
    //Wrapper function to call appropriate impl IsConnectedFunc based on CfgType
    virtual bool IsConnected(
        Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false, bool _saveFailedPath = false) {
      //clear lpOutput
      _lpOutput->Clear();
      bool connected = IsConnectedFunc<CfgType>(_env, _stats, _dm, _c1, _c2,
          _col, _lpOutput, _positionRes, _orientationRes, _checkCollision,
          _savePath, _saveFailedPath);
      if(connected)
        _lpOutput->SetLPLabel(this->GetLabel());
      return connected;
    }

    // Default for non closed chains
    template<typename Enable>
    bool IsConnectedFunc(
        Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false, bool _saveFailedPath = false,
        typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy = 0);

    // Specialization for closed chains
    template<typename Enable>
    bool IsConnectedFunc(
        Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false, bool _saveFailedPath = false,
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
        Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false, bool _saveFailedPath = false);

    /**
     * Check if two Cfgs could be connected by straight line
     * This method uses binary search to check clearances of Cfgs between _c1 and
     * _c2. If any Cfg with clearance less than 0.001 was found, false will be returned.
     */
    virtual bool IsConnectedSLBinary(
        Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false, bool _saveFailedPath = false);

    string m_vcLabel;
    bool m_binaryEvaluation;
};

template<class MPTraits>
StraightLine<MPTraits>::StraightLine(string _vcLabel, bool _evalation) :
    LocalPlannerMethod<MPTraits>(), m_vcLabel(_vcLabel), m_binaryEvaluation(_evalation) {
  this->SetName("StraightLine");
}

template<class MPTraits>
StraightLine<MPTraits>::StraightLine(MPProblemType* _problem, XMLNodeReader& _node) :
    LocalPlannerMethod<MPTraits>(_problem, _node) {
  this->SetName("StraightLine");
  m_binaryEvaluation = _node.boolXMLParameter("binaryEvaluation", false, false,
      "binary evalution along the edge");
  m_vcLabel = _node.stringXMLParameter("vcLabel", true, "",
      "Validity Test Method");
}

template<class MPTraits>
StraightLine<MPTraits>::~StraightLine() { }

template<class MPTraits>
void
StraightLine<MPTraits>::PrintOptions(ostream& _os) const {
  _os << "    " << this->GetNameAndLabel() << "::  "
      << "binary evaluation = " << " "
      << (m_binaryEvaluation ? "true" : "false") << " "
      << "vcLabel = " << " " << m_vcLabel << " "
      << endl;
}

//// default implementation for non closed chains
template<class MPTraits>
template <typename Enable>
bool
StraightLine<MPTraits>::IsConnectedFunc(
    Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath,
    typename boost::disable_if<IsClosedChain<Enable> >::type* _dummy) {

  _stats.IncLPAttempts(this->GetNameAndLabel());
  int cdCounter = 0;

  bool connected;
  if(m_binaryEvaluation)
    connected = IsConnectedSLBinary(_env, _stats, _dm, _c1, _c2, _col, _lpOutput,
        cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath);
  else
    connected = IsConnectedSLSequential(_env, _stats, _dm, _c1, _c2, _col, _lpOutput,
        cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath);
  if(connected)
    _stats.IncLPConnections(this->GetNameAndLabel());

  _stats.IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  return connected;
}

//// specialized implementation for closed chains
template<class MPTraits>
template <typename Enable>
bool
StraightLine<MPTraits>::IsConnectedFunc(
    Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath,
    typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy) {

  ValidityCheckerPointer vcm =
      this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  string callee = this->GetName() + "::IsConnectedSLSequential";

  _stats.IncLPAttempts(this->GetNameAndLabel());
  int cdCounter = 0;

  bool connected;
  if(CfgType::OrientationsDifferent(_c1, _c2)) {
    CfgType intermediate;
    bool success = intermediate.GetIntermediate(_c1, _c2);
    if(_checkCollision){
      cdCounter++;
      if(!_env->InBounds(intermediate) ||
          !vcm->IsValid(intermediate, callee)) {
        if(_env->InBounds(intermediate))
          _col = intermediate;
        return false;
      }
    }
    if(!success)
      return false;

    if(m_binaryEvaluation) {
      connected = (IsConnectedSLBinary(_env, _stats, _dm, _c1, intermediate, _col, _lpOutput,
            cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath)
          &&
          IsConnectedSLBinary(_env, _stats, _dm, intermediate, _c2, _col, _lpOutput,
            cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath)
          );
    } else {
      connected = (IsConnectedSLSequential(_env, _stats, _dm, _c1, intermediate, _col, _lpOutput,
            cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath)
          &&
          IsConnectedSLSequential(_env, _stats, _dm, intermediate, _c2, _col, _lpOutput,
            cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath)
          );
    }
  } else {
    if(m_binaryEvaluation) {
      connected = IsConnectedSLBinary(_env, _stats, _dm, _c1, _c2, _col, _lpOutput,
          cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath);
    } else {
      connected = IsConnectedSLSequential(_env, _stats, _dm, _c1, _c2, _col, _lpOutput,
          cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath);
    }
  }
  if(connected)
    _stats.IncLPConnections(this->GetNameAndLabel());

  _stats.IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  return connected;
}

template<class MPTraits>
bool
StraightLine<MPTraits>::IsConnectedSLSequential(
    Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) {

  ValidityCheckerPointer vcm =
      this->GetMPProblem()->GetValidityChecker(m_vcLabel);

  int nTicks;
  CfgType tick;
  tick = _c1;
  CfgType incr;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes, _env->GetRdRes());
#else
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);
#endif
  string callee = this->GetName() + "::IsConnectedSLSequential";

  int nIter = 0;
  for(int i = 1; i < nTicks; i++){ //don't need to check the ends, _c1 and _c2
    tick += incr;
    _cdCounter++;
    if(_checkCollision){
      if(!_env->InBounds(tick) || !vcm->IsValid(tick, callee)) {
        if(_env->InBounds(tick))
          _col = tick;
        CfgType negIncr = -incr;
        tick += negIncr;
        _lpOutput->m_edge.first.SetWeight(_lpOutput->m_edge.first.GetWeight() + nIter);
        _lpOutput->m_edge.second.SetWeight(_lpOutput->m_edge.second.GetWeight() + nIter);
        typename LPOutput<MPTraits>::LPSavedEdge tmp;
        tmp.first.first = _c1;
        tmp.first.second = tick;
        tmp.second.first = _lpOutput->m_edge.first;
        tmp.second.second = _lpOutput->m_edge.second;
        _lpOutput->m_savedEdge.push_back(tmp);
        return false;
      }
    }
    if(_savePath || _saveFailedPath) {
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
    Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) {

  ValidityCheckerPointer vcm =
    this->GetMPProblem()->GetValidityChecker(m_vcLabel);

  if(!_checkCollision)
    return IsConnectedSLSequential(_env, _stats, _dm, _c1, _c2, _col, _lpOutput,
        _cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath);

  string callee = this->GetName() + "::IsConnectedSLBinary";

  int nTicks;
  CfgType incr;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes, _env->GetRdRes());
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

    if(!_env->InBounds(midCfg) || !vcm->IsValid(midCfg, callee) ) {
      if(_env->InBounds(midCfg))
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

  if(_savePath || _saveFailedPath) {
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
