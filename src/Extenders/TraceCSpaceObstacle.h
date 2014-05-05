/*
 * =============================================================================
 *
 *       Filename:  TraceCSpaceObstacle.h
 *
 *    Description:  Trace C-space obstacle. In this growth method, we try to
 *                  find a vector parallel to a C-Space obstacle boundary. It is
 *                  not feasible to compute the actual C-Space boundary, so here
 *                  we approximate it. We first shoot a small number of rays
 *                  (our current results use two) within a gaussian distance d,
 *                  of each other, then find their collision configurations
 *                  xcol1 and xcol2. The C-obstacle vector is calculated as the
 *                  vector connecting the colliding configurations:
 *                  COV = xcol2 −xcol1 . In this way x'rand = xnear + COV.
 *
 * =============================================================================
 */
#ifndef TRACECSPACEOBSTACLE_H_
#define TRACECSPACEOBSTACLE_H_

#include "BasicExtender.h"

template<class MPTraits>
class TraceCSpaceObstacle : public BasicExtender<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    TraceCSpaceObstacle(const string& _dmLabel = "", const string& _vcLabel = "",
        double _delta = 1.0);
    TraceCSpaceObstacle(MPProblemType* _problem, XMLNodeReader& _node);

    virtual bool Extend(const CfgType& _near, const CfgType& _dir,
        CfgType& _new, vector<CfgType>& _innerNodes);
};

template<class MPTraits>
TraceCSpaceObstacle<MPTraits>::TraceCSpaceObstacle(const string& _dmLabel,
    const string& _vcLabel, double _delta) :
  BasicExtender<MPTraits>(_dmLabel, _vcLabel, _delta) {
    this->SetName("TraceCSpaceObstacle");
  }

template<class MPTraits>
TraceCSpaceObstacle<MPTraits>::TraceCSpaceObstacle(MPProblemType* _problem, XMLNodeReader& _node) :
  BasicExtender<MPTraits>(_problem, _node) {
    this->SetName("TraceCSpaceObstacle");
  }

template<class MPTraits>
bool
TraceCSpaceObstacle<MPTraits>::Extend(const CfgType& _near, const CfgType& _dir,
    CfgType& _new, vector<CfgType>& _innerNodes) {
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);
  CfgType innerCfg, rDir, cspaceDir, newNear;
  //VECTOR SCALE - THIS WILL BE HARD CODED BUT SHOULD PROBABLY BE MADE AN OPTION
  double vecScale = 1.0;
  int weight;

  // expand to c1
  if(this->m_debug)
    cout << "\texpand c1" << endl;
  if(!this->Expand(_near, _dir, innerCfg, this->m_delta, weight,
      env->GetPositionRes(), env->GetOrientationRes()))
    return false;
  _innerNodes.push_back(innerCfg);

  // expand to c2
  if(this->m_debug)
    cout << "\texpand c2" << endl;
  rDir = _dir;
  rDir.GetRandomRay(vecScale, env, dm);
  if(this->Expand(_near, rDir, innerCfg, this->m_delta, weight,
      env->GetPositionRes(), env->GetOrientationRes()))
    return false;

  // subtract newCfg2 and newCfg1 to get cspace vec
  if(this->m_debug)
    cout << "\tsubstract c2 and c1 to get c-space vec" << endl;
  if(DRand() < 0.5)
    cspaceDir = _innerNodes.back() - _innerNodes[_innerNodes.size() - 2];
  else
    cspaceDir = _innerNodes[_innerNodes.size() - 2] - _innerNodes.back();
  cspaceDir += _near;
  newNear = _near;
  dm->ScaleCfg(vecScale, cspaceDir, newNear);

  // final expand
  if(this->m_debug)
    cout << "\tfinal expand" << endl;
  if(this->Expand(newNear, cspaceDir, _new, this->m_delta, weight,
      env->GetPositionRes(), env->GetOrientationRes())) {
    _innerNodes.push_back(innerCfg);
    return true;
  }
  else {
    _new = innerCfg;
    return true;
  }
}

#endif
