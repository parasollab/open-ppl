#ifndef STRAIGHT_LINE_H_
#define STRAIGHT_LINE_H_

#include "LocalPlannerMethod.h"

#include <deque>

#include "LPOutput.h"
#include "MPProblem/IsClosedChain.h"
#include <boost/utility/enable_if.hpp>


////////////////////////////////////////////////////////////////////////////////
/// Check a straight-line path in c-space for valididty.
///
/// This local planner validates straight line paths which is a direct linear
/// interpolation between two configurations in @cspace.
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class StraightLine : public LocalPlannerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType    CfgType;
    typedef typename MPTraits::WeightType WeightType;

    ///@}
    ///@name Construction
    ///@{

    StraightLine(const std::string& _vcLabel = "", bool _binary = false,
        bool _saveIntermediates = false);

    StraightLine(XMLNode& _node);

    virtual ~StraightLine() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name LocalPlannerMethod Overrides
    ///@{

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false) override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Default for non closed chains
    bool IsConnectedFunc(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    /// Specialization for closed chains
    /// @deadcode
    /// This function is ancient and needs to be thoroughly validated before we
    /// use it again.
    //template <typename Enable>
    //bool IsConnectedFunc(
    //    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    //    LPOutput<MPTraits>* _lpOutput,
    //    double _positionRes, double _orientationRes,
    //    bool _checkCollision = true, bool _savePath = false,
    //    typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy = 0);

    /// Check if two Cfgs could be connected by straight line.
    /// This method implements straight line connection local planner
    /// by checking collision of each Cfg along the line.
    /// If the is any Cfg causes Robot collides with any obstacle,
    /// false will be returned.
    virtual bool IsConnectedSLSequential(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    /// Check if two Cfgs could be connected by straight line
    /// This method uses binary search to check clearances of Cfgs between _c1
    /// and _c2. If any Cfg with clearance less than 0.001 was found, false will
    /// be returned.
    virtual bool IsConnectedSLBinary(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_vcLabel;
    bool m_binaryEvaluation{false}; ///< Use binary search?

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
StraightLine<MPTraits>::
StraightLine(const string& _vcLabel, bool _binary, bool _saveIntermediates)
  : LocalPlannerMethod<MPTraits>(_saveIntermediates),
    m_vcLabel(_vcLabel), m_binaryEvaluation(_binary) {
  this->SetName("StraightLine");
}


template <typename MPTraits>
StraightLine<MPTraits>::
StraightLine(XMLNode& _node) : LocalPlannerMethod<MPTraits>(_node) {
  this->SetName("StraightLine");

  m_binaryEvaluation = _node.Read("binaryEvaluation", false, m_binaryEvaluation,
      "Use binary search to evaluate the edge, or linear scan if false.");

  m_vcLabel = _node.Read("vcLabel", true, "", "The validity checker to use.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
StraightLine<MPTraits>::
Print(std::ostream& _os) const {
  LocalPlannerMethod<MPTraits>::Print(_os);
  _os << "\tbinary evaluation = " << (m_binaryEvaluation ? "true" : "false")
      << "\n\tvcLabel = " << m_vcLabel
      << std::endl;
}

/*----------------------- LocalPlannerMethod Overrides -----------------------*/

template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnected(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
  if(this->m_debug)
    std::cout << this->GetName() << "::IsConnected"
              << "\n\tChecking line from " << _c1.PrettyPrint()
              << " to " << _c2.PrettyPrint()
              << "\n\tUsing " << (m_binaryEvaluation ? "binary" : "sequential")
              << " evaluation."
              << std::endl;

  _lpOutput->Clear();
  bool connected = IsConnectedFunc(_c1, _c2,
      _col, _lpOutput, _positionRes, _orientationRes, _checkCollision,
      _savePath);
  if(connected)
    _lpOutput->SetLPLabel(this->GetLabel());

  if(this->m_debug)
    std::cout << "\n\tLocal Plan is "
              << (connected ? "valid" : "invalid at " + _col.PrettyPrint())
              << std::endl;
  return connected;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnectedFunc(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {

  StatClass* stats = this->GetStatClass();

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


//template <typename MPTraits>
//template <typename Enable>
//bool
//StraightLine<MPTraits>::
//IsConnectedFunc(
//    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
//    LPOutput<MPTraits>* _lpOutput,
//    double _positionRes, double _orientationRes,
//    bool _checkCollision, bool _savePath,
//    typename boost::enable_if<IsClosedChain<Enable> >::type* _dummy) {
//  /// @note Specialized implementation for closed chains
//
//  Environment* env = this->GetEnvironment();
//  auto vc = this->GetValidityChecker(m_vcLabel);
//  StatClass* stats = this->GetStatClass();
//  string callee = this->GetNameAndLabel() + "::IsConnectedSLSequential";
//
//  stats->IncLPAttempts(this->GetNameAndLabel());
//  int cdCounter = 0;
//
//  bool connected;
//  if(CfgType::OrientationsDifferent(_c1, _c2)) {
//    CfgType intermediate(this->GetTask()->GetRobot());
//    bool success = intermediate.GetIntermediate(_c1, _c2);
//    if(_checkCollision) {
//      cdCounter++;
//      if(!intermediate.InBounds(env) ||
//          !vc->IsValid(intermediate, callee)) {
//        if(intermediate.InBounds(env))
//          _col = intermediate;
//        return false;
//      }
//    }
//    if(!success)
//      return false;
//
//    if(m_binaryEvaluation) {
//      connected = (IsConnectedSLBinary(_c1, intermediate, _col, _lpOutput,
//            cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath)
//          and
//          IsConnectedSLBinary(intermediate, _c2, _col, _lpOutput,
//            cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath)
//          );
//    }
//    else {
//      connected = (IsConnectedSLSequential(_c1, intermediate, _col, _lpOutput,
//            cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath)
//          and
//          IsConnectedSLSequential(intermediate, _c2, _col, _lpOutput,
//            cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath)
//          );
//    }
//  }
//  else {
//    if(m_binaryEvaluation)
//      connected = IsConnectedSLBinary(_c1, _c2, _col, _lpOutput,
//          cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath);
//    else
//      connected = IsConnectedSLSequential(_c1, _c2, _col, _lpOutput,
//          cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath);
//  }
//  if(connected)
//    stats->IncLPConnections(this->GetNameAndLabel());
//
//  stats->IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
//  return connected;
//}


template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnectedSLSequential(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {

  Environment* env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();
  auto vc = this->GetValidityChecker(m_vcLabel);

  int nTicks;
  CfgType tick(robot), incr(robot);
  tick = _c1;
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);
  string callee = this->GetNameAndLabel() + "::IsConnectedSLSequential";

  if(this->m_debug)
    std::cout << "\n\tComputed increment for " << nTicks << " ticks: "
              << incr.PrettyPrint()
              << std::endl;

  int nIter = 0;
  for(int i = 1; i < nTicks; i++) { //don't need to check the ends, _c1 and _c2
    tick += incr;
    _cdCounter++;
    if(_checkCollision) {
      if(this->m_debug)
        std::cout << "\n\t\tChecking " << tick.PrettyPrint();

      const bool inBounds = tick.InBounds(env);
      if(!inBounds || !vc->IsValid(tick, callee)) {
        /// @TODO This looks wrong - we should be setting _col regardless of
        ///       whether we hit the boundary or obstacle.
        if(inBounds)
          _col = tick;
        CfgType negIncr = -incr;
        tick += negIncr;
        _lpOutput->m_edge.first.SetWeight(_lpOutput->m_edge.first.GetWeight()
            + nIter);
        _lpOutput->m_edge.second.SetWeight(_lpOutput->m_edge.second.GetWeight()
            + nIter);

        if(this->m_debug)
          std::cout << " INVALID";
        return false;
      }
      else if(this->m_debug)
        std::cout << " OK";
    }
    if(_savePath)
      _lpOutput->m_path.push_back(tick);
    nIter++;
  }

  _lpOutput->m_edge.first.SetWeight(_lpOutput->m_edge.first.GetWeight() + nIter);
  _lpOutput->m_edge.second.SetWeight(_lpOutput->m_edge.second.GetWeight() + nIter);

  return true;
}


template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnectedSLBinary(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {

  Environment* env = this->GetEnvironment();
  auto vc = this->GetValidityChecker(m_vcLabel);

  if(!_checkCollision)
    return IsConnectedSLSequential(_c1, _c2, _col, _lpOutput,
        _cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath);

  string callee = this->GetNameAndLabel() + "::IsConnectedSLBinary";

  int nTicks;
  CfgType incr(this->GetTask()->GetRobot());
  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);

  if(this->m_debug)
    std::cout << "\n\tComputed increment for " << nTicks << " ticks: "
              << incr.PrettyPrint()
              << std::endl;

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

    if(this->m_debug)
      std::cout << "\n\t\tChecking " << midCfg.PrettyPrint();

    _cdCounter++;

    if(!midCfg.InBounds(env) || !vc->IsValid(midCfg, callee) ) {
      if(midCfg.InBounds(env))
        _col = midCfg;
      if(this->m_debug)
        std::cout << " INVALID";
      return false;
    }
    else {
      if(i + 1 != mid)
        Q.push_back(make_pair(i, mid));
      if(mid + 1 != j)
        Q.push_back(make_pair(mid, j));
      if(this->m_debug)
        std::cout << " OK";
    }
  }

  if(_savePath) {
    CfgType tick = _c1;
    for(int n = 1; n < nTicks; ++n) {
      tick += incr;
      _lpOutput->m_path.push_back(tick);
    }
  }

  _lpOutput->m_edge.first.SetWeight(_lpOutput->m_edge.first.GetWeight()
      + nTicks - 1);
  _lpOutput->m_edge.second.SetWeight(_lpOutput->m_edge.second.GetWeight()
      + nTicks - 1);

  return true;
}

/*----------------------------------------------------------------------------*/

#endif
