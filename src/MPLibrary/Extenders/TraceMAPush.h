#ifndef PMPL_TRACE_MA_PUSH_H_
#define PMPL_TRACE_MA_PUSH_H_

#include "TraceObstacle.h"


////////////////////////////////////////////////////////////////////////////////
/// Extend tangent to a workspace obstacle and then push to the medial axis.
///
/// @todo This object needs to have its dedicated MA tool removed and replaced
///       with a label (fetch from MPTools).
///
/// This performs the same as @c TraceObstacle but after the target
/// configuration has been pushed in the obstacle direction, it is then pushed
/// toward the medial axis of @cfree. In this way \f$q_{near}\f$ is extended
/// toward a \f$q_{dir}\f$ which is near the medial axis.
///
/// @ingroup Extenders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TraceMAPush : public TraceObstacle<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    TraceMAPush();

    TraceMAPush(XMLNode& _node);

    virtual ~TraceMAPush() = default;

    ///@}
    ///@name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    /// A utility object for pushing configurations to the medial axis.
    MedialAxisUtility<MPTraits> m_medialAxisUtility;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
TraceMAPush<MPTraits>::
TraceMAPush() {
  this->SetName("TraceMAPush");
}


template <typename MPTraits>
TraceMAPush<MPTraits>::
TraceMAPush(XMLNode& _node) : TraceObstacle<MPTraits>(_node),
    m_medialAxisUtility(_node) {
  this->SetName("TraceMAPush");
}

/*-------------------------- ExtenderMethod Overrides ------------------------*/

template <typename MPTraits>
bool
TraceMAPush<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end,
    CfgType& _new, LPOutput<MPTraits>& _lp) {
  // Setup MP Variables
  Environment* env = this->GetEnvironment();
  CfgType innerCfg(this->GetTask()->GetRobot());

  // The target configuration is pushed in the obstacle direction
  TraceObstacle<MPTraits>::Extend(_start, _end, innerCfg, _lp);
  _lp.m_intermediates.push_back(innerCfg);

  // The target cfg is pushed toward the medial axis of the configuration space
  if(this->m_debug)
    cout << "pushed toward the medial axis" << endl;
  if(m_medialAxisUtility.PushToMedialAxis(innerCfg,
      this->GetEnvironment()->GetBoundary())) {
    LPOutput<MPTraits> newLPOutput;
    bool result = this->Expand(_start, innerCfg, _new, this->m_maxDist,
        newLPOutput, env->GetPositionRes(), env->GetOrientationRes());
    _lp.m_edge.first.SetWeight(_lp.m_edge.first.GetWeight() +
        newLPOutput.m_edge.first.GetWeight());
    _lp.m_edge.second.SetWeight(_lp.m_edge.second.GetWeight() +
        newLPOutput.m_edge.second.GetWeight());
    return result;
  } else
    return false;
}

/*----------------------------------------------------------------------------*/

#endif
