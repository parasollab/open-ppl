#ifndef BASIC_EXTENDER_H_
#define BASIC_EXTENDER_H_

#include "ExtenderMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Basic straight-line extension.
///
/// Extends in straight-line through @cspace from \f$q_{near}\f$ towards
/// \f$q_{dir}\f$ until either \f$q_{dir}\f$ is reached, a distance of
/// \f$\Delta q\f$ is extended, or @cobst is reached.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class BasicExtender : public ExtenderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    BasicExtender(const string& _dmLabel = "", const string& _vcLabel = "",
        double _min = .001, double _max = 1, bool _randomOrientation = true);

    BasicExtender(XMLNode& _node);

    virtual ~BasicExtender() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) override;

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
           CfgType& _new, LPOutput<MPTraits>& _lp, CDInfo& _cdInfo) override;

    ///@}
    ///@name Helpers
    ///@{

    /// Basic utility for "extend" a RRT tree. Assumed to be given a start node
    /// and a goal node to grow towards. Resulting node extended towards the
    /// goal is passed by reference and modified.
    /// @param _start  Cfg to grow from.
    /// @param _end    Cfg to grow toward.
    /// @param _newCfg Return for newly created cfg.
    /// @param _delta  Maximum distance to grow
    /// @return True if the extension produced a valid configuration that is at
    ///         least the minimum distance away from the starting point.
    bool Expand(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
        double _delta, LPOutput<MPTraits>& _lp,
        double _posRes, double _oriRes);
    bool Expand(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
        double _delta, LPOutput<MPTraits>& _lp, CDInfo& _cdInfo,
        double _posRes, double _oriRes);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    string m_dmLabel;         ///< The distance metric to use.
    string m_vcLabel;         ///< The validity checker to use.
    bool m_randomOrientation; ///< Setting this to false fixes orientation.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
BasicExtender<MPTraits>::
BasicExtender(const string& _dmLabel, const string& _vcLabel, double _min,
    double _max, bool _randomOrientation) :
    ExtenderMethod<MPTraits>(_min, _max), m_dmLabel(_dmLabel),
    m_vcLabel(_vcLabel), m_randomOrientation(_randomOrientation) {
  this->SetName("BasicExtender");
}


template <typename MPTraits>
BasicExtender<MPTraits>::
BasicExtender(XMLNode& _node) : ExtenderMethod<MPTraits>(_node) {
  this->SetName("BasicExtender");

  m_dmLabel = _node.Read("dmLabel", true, "", "Distance metric label");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity checker label");
  m_randomOrientation = _node.Read("randomOrientation", false, true,
      "Random orientation");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
BasicExtender<MPTraits>::
Print(ostream& _os) const {
  ExtenderMethod<MPTraits>::Print(_os);
  _os << "\tdistance metric : \"" << m_dmLabel << "\"" << endl
      << "\tvalidity checker : \"" << m_vcLabel << "\"" << endl
      << "\trandom orientation : " << (m_randomOrientation ? "y" : "n") << endl;
}

/*------------------------- ExtenderMethod Overrides -------------------------*/

template <typename MPTraits>
bool
BasicExtender<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end, CfgType& _new,
       LPOutput<MPTraits>& _lp) {
  Environment* env = this->GetEnvironment();

  // If non-random orientation, adjust the end's non-positional DOFs to match
  // the start.
  if(!m_randomOrientation) {
    CfgType end = _end;
    for(size_t i = end.PosDOF(); i < _end.DOF(); i++)
      end[i] = _start[i];
    return Expand(_start, end, _new, this->m_maxDist, _lp,
        env->GetPositionRes(), env->GetOrientationRes());
  }
  return Expand(_start, _end, _new, this->m_maxDist, _lp,
      env->GetPositionRes(), env->GetOrientationRes());
}


template <typename MPTraits>
bool
BasicExtender<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end,
       CfgType& _new, LPOutput<MPTraits>& _lp, CDInfo& _cdInfo) {
  Environment* env = this->GetEnvironment();

  //Assume that all CD info is wanted, and clear out all data:
  _cdInfo.ResetVars(true);

  // If non-random orientation, adjust the end's non-positional DOFs to match
  // the start.
  if(!m_randomOrientation) {
    CfgType end = _end;
    for(size_t i = end.PosDOF(); i < _end.DOF(); i++)
      end[i] = _start[i];
    return Expand(_start, end, _new, this->m_maxDist, _lp, _cdInfo,
                  env->GetPositionRes(), env->GetOrientationRes());
  }
  return Expand(_start, _end, _new, this->m_maxDist, _lp, _cdInfo,
                env->GetPositionRes(), env->GetOrientationRes());
}

/*-------------------------------- Helpers? ----------------------------------*/

template <typename MPTraits>
bool
BasicExtender<MPTraits>::
Expand(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
    double _delta, LPOutput<MPTraits>& _lp, double _posRes, double _oriRes) {
  CDInfo cdInfo;
  return Expand(_start, _end, _newCfg, _delta, _lp, cdInfo, _posRes, _oriRes);
}


template <typename MPTraits>
bool
BasicExtender<MPTraits>::
Expand(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
    double _delta, LPOutput<MPTraits>& _lp, CDInfo& _cdInfo,
    double _posRes, double _oriRes) {
  _lp.Clear();
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto vc = this->GetValidityChecker(m_vcLabel);
  const std::string callee("BasicExtender::Expand");

  CfgType incr(this->GetTask()->GetRobot()), tick = _start, previous = _start;
  bool collision = false;
  int nTicks, ticker = 0;

  incr.FindIncrement(tick, _end, &nTicks, _posRes, _oriRes);

  if(this->m_debug)
    std::cout << "Trying extension:"
              << "\n\tFrom: " << _start.PrettyPrint()
              << "\n\tTo:   " << _end.PrettyPrint()
              << "\n\tIncr: " << incr.PrettyPrint()
              << "\n\tNum ticks: " << nTicks
              << std::endl;

  // Move out from start towards dir, bounded by number of ticks allowed at a
  // given resolution and the distance _delta: the maximum distance to grow
  while(!collision && dm->Distance(_start, tick) <= _delta &&
        ticker <= nTicks) {
    previous = tick;
    tick += incr;
    if(!vc->IsValid(tick, _cdInfo, callee))
      collision = true; //return previous tick, as it is collision-free
    ++ticker;
  }

  // Quit if we didn't expand at all.
  if(previous == _start) {
    if(this->m_debug)
      std::cout << "Could not expand !" << std::endl;
    return false;
  }

  // If we did expand, set _newCfg to the end of the extension.
  if(ticker == nTicks + 1)
    // Full expansion. We have to adjust _newCfg to be equal to _end because
    // of accumulated floating-point error from the division in FindIncrement and
    // adding incr to tick.
    // We do not specifically collision check _end because previous is within a
    // resolution of _end (they are very, very close) and it was already checked.
    _newCfg = _end;
  else
    // Collision reached. Use previous as it is the last collision-free tick
    _newCfg = previous;

  // Set edge weight according to distance metric.
  const double distance = dm->Distance(_start, _newCfg);
  _lp.m_edge.first.SetWeight(distance);
  _lp.m_edge.second.SetWeight(distance);

  if(this->m_debug)
    std::cout << "Extended: " << std::setprecision(4) << distance << " units."
              << "\n\tMin:Max distance: "
              << this->GetMinDistance() << " : " << this->GetMaxDistance()
              << "\n\tend: " << _newCfg.PrettyPrint()
              << std::endl;

  return distance >= this->m_minDist;
}

/*----------------------------------------------------------------------------*/

#endif
