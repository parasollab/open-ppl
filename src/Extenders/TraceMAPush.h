#ifndef TRACE_MA_PUSH_H_
#define TRACE_MA_PUSH_H_

#include "TraceObstacle.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Extend tangent to a workspace obstacle and then push to the medial axis.
/// @tparam MPTraits Motion planning universe
///
/// This performs the same as @c TraceObstacle but after the target
/// configuration has been pushed in the obstacle direction, it is then pushed
/// toward the medial axis of @cfree. In this way \f$q_{near}\f$ is extended
/// toward a \f$q_{dir}\f$ which is near the medial axis.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class TraceMAPush : public TraceObstacle<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    TraceMAPush(const string& _dmLabel = "", const string& _vcLabel = "",
        double _delta = 1.0);
    TraceMAPush(MPProblemType* _problem, XMLNode& _node);

    virtual bool Extend(const CfgType& _near, const CfgType& _dir,
        CfgType& _new, LPOutput<MPTraits>& _lpOutput);

  private:
    MedialAxisUtility<MPTraits> m_medialAxisUtility;
};

template<class MPTraits>
TraceMAPush<MPTraits>::
TraceMAPush(const string& _dmLabel,
    const string& _vcLabel, double _delta) :
  TraceObstacle<MPTraits>(_dmLabel, _vcLabel, _delta) {
    this->SetName("TraceMAPush");
  }

template<class MPTraits>
TraceMAPush<MPTraits>::
TraceMAPush(MPProblemType* _problem, XMLNode& _node) :
  TraceObstacle<MPTraits>(_problem, _node),
  m_medialAxisUtility(_problem, _node) {
    this->SetName("TraceMAPush");
  }

template<class MPTraits>
bool
TraceMAPush<MPTraits>::
Extend(const CfgType& _near, const CfgType& _dir,
    CfgType& _new, LPOutput<MPTraits>& _lpOutput) {
  // Setup MP Variables
  Environment* env = this->GetEnvironment();
  CfgType innerCfg;

  // The target configuration is pushed in the obstacle direction
  TraceObstacle<MPTraits>::Extend(_near, _dir, innerCfg, _lpOutput);
  _lpOutput.m_intermediates.push_back(innerCfg);

  // The target cfg is pushed toward the medial axis of the configuration space
  if(this->m_debug)
    cout << "pushed toward the medial axis" << endl;
  if(m_medialAxisUtility.PushToMedialAxis(innerCfg,
      this->GetEnvironment()->GetBoundary())) {
    LPOutput<MPTraits> newLPOutput;
    bool result = this->Expand(_near, innerCfg, _new, this->m_maxDist,
        newLPOutput, env->GetPositionRes(), env->GetOrientationRes());
    _lpOutput.m_edge.first.SetWeight(_lpOutput.m_edge.first.GetWeight() +
        newLPOutput.m_edge.first.GetWeight());
    _lpOutput.m_edge.second.SetWeight(_lpOutput.m_edge.second.GetWeight() +
        newLPOutput.m_edge.second.GetWeight());
    return result;
  } else
    return false;
}

#endif
