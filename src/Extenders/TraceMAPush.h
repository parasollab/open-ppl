#ifndef TRACEMAPUSH_H_
#define TRACEMAPUSH_H_

#include "TraceObstacle.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Extend tangent to a workspace obstacle and then push to the medial axis.
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
    TraceMAPush(MPProblemType* _problem, XMLNodeReader& _node);

    virtual bool Extend(const CfgType& _near, const CfgType& _dir,
        CfgType& _new, vector<CfgType>& _innerNodes);

  private:
    MedialAxisUtility<MPTraits> m_medialAxisUtility;
};

template<class MPTraits>
TraceMAPush<MPTraits>::TraceMAPush(const string& _dmLabel,
    const string& _vcLabel, double _delta) :
  TraceObstacle<MPTraits>(_dmLabel, _vcLabel, _delta) {
  this->SetName("TraceMAPush");
}

template<class MPTraits>
TraceMAPush<MPTraits>::TraceMAPush(MPProblemType* _problem, XMLNodeReader& _node) :
  TraceObstacle<MPTraits>(_problem, _node),
  m_medialAxisUtility(_problem, _node) {
    this->SetName("TraceMAPush");
  }

template<class MPTraits>
bool
TraceMAPush<MPTraits>::Extend(const CfgType& _near, const CfgType& _dir,
    CfgType& _new, vector<CfgType>& _innerNodes) {
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CfgType innerCfg;
  int weight;

  // The target configuration is pushed in the obstacle direction
  TraceObstacle<MPTraits>::Extend(_near, _dir, innerCfg, _innerNodes);

  // The target cfg is pushed toward the medial axis of the configuration space
  if(this->m_debug)
    cout << "pushed toward the medial axis" << endl;
  if(m_medialAxisUtility.PushToMedialAxis(innerCfg,
      this->GetMPProblem()->GetEnvironment()->GetBoundary()))
    return this->Expand(_near, innerCfg, _new, this->m_delta, weight,
        env->GetPositionRes(), env->GetOrientationRes());
  else
    return false;
}

#endif
