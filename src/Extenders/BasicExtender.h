#ifndef BASIC_EXTENDER_H_
#define BASIC_EXTENDER_H_

#include "ExtenderMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Basic straight-line extension.
/// @tparam MPTraits Motion planning universe
///
/// Extends in straight-line through @cspace from \f$q_{near}\f$ towards
/// \f$q_{dir}\f$ until either \f$q_{dir}\f$ is reached, a distance of
/// \f$\Delta q\f$ is extended, or @cobst is reached.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class BasicExtender : public ExtenderMethod<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    ///@}
    ///\name Construction
    ///@{

    BasicExtender(const string& _dmLabel = "", const string& _vcLabel = "",
        double _delta = 1.0, bool _randomOrientation = true);
    BasicExtender(MPProblemType* _problem, XMLNode& _node);
    virtual ~BasicExtender() = default;

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const override;

    ///@}
    ///\name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lpOutput);

    ///@}
    ///\name Helpers?
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Basic utility for "extend" a RRT tree. Assumed to be given a start
    ///        node and a goal node to grow towards. Resulting node extended
    ///        towards the goal is passed by reference and modified.
    /// \param[in]  _start  Cfg to grow from.
    /// \param[in]  _end    Cfg to grow toward.
    /// \param[out] _newCfg Return for newly created cfg.
    /// \param[in]  _delta  Maximum distance to grow
    bool Expand(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
        double _delta, LPOutput<MPTraits>& _lpOutput,
        double _posRes, double _oriRes);
    bool Expand(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
        double _delta, LPOutput<MPTraits>& _lpOutput, CDInfo& _cdInfo,
        double _posRes, double _oriRes);

    ///@}

  protected:

    string m_dmLabel;
    string m_vcLabel;
    bool m_randomOrientation;
};

/*------------------------------- Construction -------------------------------*/

template<class MPTraits>
BasicExtender<MPTraits>::
BasicExtender(const string& _dmLabel, const string& _vcLabel, double _delta,
    bool _randomOrientation) :
    ExtenderMethod<MPTraits>(), m_dmLabel(_dmLabel), m_vcLabel(_vcLabel),
    m_randomOrientation(_randomOrientation) {
  this->m_maxDist = _delta;
  this->SetName("BasicExtender");
}


template<class MPTraits>
BasicExtender<MPTraits>::
BasicExtender(MPProblemType* _problem, XMLNode& _node) :
    ExtenderMethod<MPTraits>(_problem, _node) {
  this->SetName("BasicExtender");
  ParseXML(_node);
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template<class MPTraits>
void
BasicExtender<MPTraits>::
ParseXML(XMLNode& _node) {
  m_dmLabel = _node.Read("dmLabel", true, "", "Distance metric label");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity checker label");
  m_randomOrientation = _node.Read("randomOrientation", false, true,
      "Random orientation");
}


template<class MPTraits>
void
BasicExtender<MPTraits>::
Print(ostream& _os) const {
  ExtenderMethod<MPTraits>::Print(_os);
  _os << "\tdistance metric : \"" << m_dmLabel << "\"" << endl
      << "\tvalidity checker : \"" << m_vcLabel << "\"" << endl
      << "\trandom orientation : " << (m_randomOrientation ? "y" : "n") << endl;
}

/*------------------------- ExtenderMethod Overrides -------------------------*/

template<class MPTraits>
bool
BasicExtender<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lpOutput) {
  Environment* env = this->GetEnvironment();

  // If non-random orientation, adjust the end's non-positional DOFs to match
  // the start.
  if(!m_randomOrientation) {
    CfgType end = _end;
    for(size_t i = end.PosDOF(); i < _end.DOF(); i++)
      end[i] = _start[i];
    return Expand(_start, end, _new, this->m_maxDist, _lpOutput,
        env->GetPositionRes(), env->GetOrientationRes());
  }
  return Expand(_start, _end, _new, this->m_maxDist, _lpOutput,
      env->GetPositionRes(), env->GetOrientationRes());
}

/*-------------------------------- Helpers? ----------------------------------*/

template<class MPTraits>
bool
BasicExtender<MPTraits>::
Expand(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
    double _delta, LPOutput<MPTraits>& _lpOutput, double _posRes,
    double _oriRes) {
  CDInfo cdInfo;
  return Expand(_start, _end, _newCfg, _delta, _lpOutput, cdInfo, _posRes,
      _oriRes);
}


template<class MPTraits>
bool
BasicExtender<MPTraits>::
Expand(const CfgType& _start, const CfgType& _end, CfgType& _newCfg,
    double _delta, LPOutput<MPTraits>& _lpOutput, CDInfo& _cdInfo,
    double _posRes, double _oriRes) {
  Environment* env = this->GetEnvironment();
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto vc = this->GetValidityChecker(m_vcLabel);
  string callee("BasicExtender::Expand");

  CfgType incr, tick = _start, previous = _start;
  bool collision = false;
  int nTicks, ticker = 0;

  incr.FindIncrement(tick,_end,&nTicks, _posRes, _oriRes);

  // Move out from start towards dir, bounded by number of ticks allowed at a
  // given resolution and the distance _delta: the maximum distance to grow
  while(!collision && dm->Distance(_start, tick) <= _delta &&
        ticker <= nTicks) {
    previous = tick;
    tick += incr;
    if(!env->InBounds(tick) || !(vc->IsValid(tick, _cdInfo, callee)))
      collision = true; //return previous tick, as it is collision-free
    ++ticker;
  }

  // Quit if we didn't expand at all.
  if(previous == _start) {
    if(this->m_debug)
      cout << "Could not expand !" << endl;
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
  double distance = dm->Distance(_start, _newCfg);
  _lpOutput.m_edge.first.SetWeight(distance);
  _lpOutput.m_edge.second.SetWeight(distance);

  return true;
}

/*----------------------------------------------------------------------------*/

#endif
