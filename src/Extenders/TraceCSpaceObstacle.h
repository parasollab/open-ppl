#ifndef TRACECSPACEOBSTACLE_H_
#define TRACECSPACEOBSTACLE_H_

#include "BasicExtender.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Extend in a direction tangent to @cobst.
///
/// Trace @cobst. In this growth method, we try to find a vector tangent to a
/// @coboundary. It is not feasible to compute @coboundary, so here we
/// approximate it. We first shoot a small number of rays (our current results
/// use two) within a gaussian distance d, of each other, then find their
/// collision configurations \f$q_1\f$ and \f$q_2\f$. The @cobst vector is
/// calculated as the vector connecting the colliding configurations:
/// \f$C = q_2 - q_1\f$. In this way \f$q_{dir} = q_{near} + C\f$.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class TraceCSpaceObstacle : public BasicExtender<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    TraceCSpaceObstacle(const string& _dmLabel = "",
        const string& _vcLabel = "", double _delta = 1.0);
    TraceCSpaceObstacle(MPProblemType* _problem, XMLNode& _node);

    virtual bool Extend(const CfgType& _near, const CfgType& _dir,
        CfgType& _new, LPOutput<MPTraits>& _lpOutput);
};

template<class MPTraits>
TraceCSpaceObstacle<MPTraits>::TraceCSpaceObstacle(const string& _dmLabel,
    const string& _vcLabel, double _delta) :
  BasicExtender<MPTraits>(_dmLabel, _vcLabel, _delta) {
    this->SetName("TraceCSpaceObstacle");
  }

template<class MPTraits>
TraceCSpaceObstacle<MPTraits>::TraceCSpaceObstacle(MPProblemType* _problem,
    XMLNode& _node) :
  BasicExtender<MPTraits>(_problem, _node) {
    this->SetName("TraceCSpaceObstacle");
  }

template<class MPTraits>
bool
TraceCSpaceObstacle<MPTraits>::Extend(const CfgType& _near, const CfgType& _dir,
    CfgType& _new, LPOutput<MPTraits>& _lpOutput) {
  // Setup MP Variables
  Environment* env = this->GetEnvironment();
  DistanceMetricPointer dm = this->GetDistanceMetric(this->m_dmLabel);
  CfgType innerCfg, rDir, cspaceDir, newNear;
  //VECTOR SCALE - THIS WILL BE HARD CODED BUT SHOULD PROBABLY BE MADE AN OPTION
  double vecScale = 1.0;

  // expand to c1
  if(this->m_debug)
    cout << "\texpand c1" << endl;
  if(!this->Expand(_near, _dir, innerCfg, this->m_delta, _lpOutput,
      env->GetPositionRes(), env->GetOrientationRes()))
    return false;
  _lpOutput.m_intermediates.push_back(innerCfg);

  // expand to c2
  if(this->m_debug)
    cout << "\texpand c2" << endl;
  rDir.GetRandomRay(vecScale, dm);
  if(!this->Expand(_near, rDir, innerCfg, this->m_delta, _lpOutput,
      env->GetPositionRes(), env->GetOrientationRes()))
    return false;
  _lpOutput.m_intermediates.push_back(innerCfg);

  // subtract newCfg2 and newCfg1 to get cspace vec
  if(this->m_debug)
    cout << "\tsubstract c2 and c1 to get c-space vec" << endl;
  if(DRand() < 0.5)
    cspaceDir = _lpOutput.m_intermediates.back() -
        _lpOutput.m_intermediates[_lpOutput.m_intermediates.size() - 2];
  else
    cspaceDir = _lpOutput.m_intermediates[_lpOutput.m_intermediates.size() - 2]
        - _lpOutput.m_intermediates.back();
  cspaceDir += _near;
  newNear = _near;
  dm->ScaleCfg(vecScale, cspaceDir, newNear);

  // final expand
  if(this->m_debug)
    cout << "\tfinal expand" << endl;
  if(this->Expand(newNear, cspaceDir, _new, this->m_delta, _lpOutput,
      env->GetPositionRes(), env->GetOrientationRes())) {
    _lpOutput.m_intermediates.push_back(innerCfg);
    return true;
  }
  else {
    _new = innerCfg;
    return true;
  }
}

#endif
