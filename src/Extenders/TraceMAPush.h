#ifndef TRACE_MA_PUSH_H_
#define TRACE_MA_PUSH_H_

#include "TraceObstacle.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Extend tangent to a workspace obstacle and then push to the medial
///        axis.
/// @tparam MPTraits Motion planning universe
///
/// This performs the same as @c TraceObstacle but after the target
/// configuration has been pushed in the obstacle direction, it is then pushed
/// toward the medial axis of @cfree. In this way \f$q_{near}\f$ is extended
/// toward a \f$q_{dir}\f$ which is near the medial axis.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TraceMAPush : public TraceObstacle<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ///@}
    ///\name Construction
    ///@{

    TraceMAPush(const string& _dmLabel = "", const string& _vcLabel = "",
        double _min = .001, double _max = 1);

    TraceMAPush(MPProblemType* _problem, XMLNode& _node);

    virtual ~TraceMAPush() = default;

    ///@}
    ///\name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp);

    ///@}

  private:

    ///\name Internal State
    ///@{

    MedialAxisUtility<MPTraits> m_medialAxisUtility;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
TraceMAPush<MPTraits>::
TraceMAPush(const string& _dmLabel, const string& _vcLabel, double _min,
    double _max) : TraceObstacle<MPTraits>(_dmLabel, _vcLabel, _min, _max) {
  this->SetName("TraceMAPush");
}


template <typename MPTraits>
TraceMAPush<MPTraits>::
TraceMAPush(MPProblemType* _problem, XMLNode& _node) :
    TraceObstacle<MPTraits>(_problem, _node),
    m_medialAxisUtility(_problem, _node) {
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
  CfgType innerCfg;

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
