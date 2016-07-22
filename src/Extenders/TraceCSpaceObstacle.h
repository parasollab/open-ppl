#ifndef TRACECSPACEOBSTACLE_H_
#define TRACECSPACEOBSTACLE_H_

#include "BasicExtender.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Extenders
/// @brief Extend in a direction tangent to @cobst.
/// @tparam MPTraits Motion planning universe
///
/// Trace @cobst. In this growth method, we try to find a vector tangent to a
/// @coboundary. It is not feasible to compute @coboundary, so here we
/// approximate it. We first shoot a small number of rays (our current results
/// use two) within a gaussian distance d, of each other, then find their
/// collision configurations \f$q_1\f$ and \f$q_2\f$. The @cobst vector is
/// calculated as the vector connecting the colliding configurations:
/// \f$C = q_2 - q_1\f$. In this way \f$q_{dir} = q_{near} + C\f$.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TraceCSpaceObstacle : public BasicExtender<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    ///@}
    ///\name Construction
    ///@{

    TraceCSpaceObstacle(const string& _dmLabel = "", const string& _vcLabel = "",
        double _min = .001, double _max = 1);

    TraceCSpaceObstacle(MPProblemType* _problem, XMLNode& _node);

    virtual ~TraceCSpaceObstacle() = default;

    ///@}
    ///\name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) override;

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
TraceCSpaceObstacle<MPTraits>::
TraceCSpaceObstacle(const string& _dmLabel, const string& _vcLabel,
    double _min, double _max) :
    BasicExtender<MPTraits>(_dmLabel, _vcLabel, _min, _max) {
  this->SetName("TraceCSpaceObstacle");
}


template <typename MPTraits>
TraceCSpaceObstacle<MPTraits>::
TraceCSpaceObstacle(MPProblemType* _problem, XMLNode& _node) :
    BasicExtender<MPTraits>(_problem, _node) {
  this->SetName("TraceCSpaceObstacle");
}

/*------------------------- ExtenderMethod Overrides -------------------------*/

template <typename MPTraits>
bool
TraceCSpaceObstacle<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp) {
  // Setup MP Variables
  Environment* env = this->GetEnvironment();
  auto dm = this->GetDistanceMetric(this->m_dmLabel);
  CfgType innerCfg, rDir, cspaceDir, newNear;
  //VECTOR SCALE - THIS WILL BE HARD CODED BUT SHOULD PROBABLY BE MADE AN OPTION
  double vecScale = 1.0;

  // expand to c1
  if(this->m_debug)
    cout << "\texpand c1" << endl;
  if(!this->Expand(_start, _end, innerCfg, this->m_maxDist, _lp,
      env->GetPositionRes(), env->GetOrientationRes()))
    return false;
  _lp.m_intermediates.push_back(innerCfg);

  // expand to c2
  if(this->m_debug)
    cout << "\texpand c2" << endl;
  rDir.GetRandomRay(vecScale, dm);
  if(!this->Expand(_start, rDir, innerCfg, this->m_maxDist, _lp,
      env->GetPositionRes(), env->GetOrientationRes()))
    return false;
  _lp.m_intermediates.push_back(innerCfg);

  // subtract newCfg2 and newCfg1 to get cspace vec
  if(this->m_debug)
    cout << "\tsubstract c2 and c1 to get c-space vec" << endl;
  if(DRand() < 0.5)
    cspaceDir = _lp.m_intermediates.back() -
        _lp.m_intermediates[_lp.m_intermediates.size() - 2];
  else
    cspaceDir = _lp.m_intermediates[_lp.m_intermediates.size() - 2]
        - _lp.m_intermediates.back();
  cspaceDir += _start;
  newNear = _start;
  dm->ScaleCfg(vecScale, cspaceDir, newNear);

  // final expand
  if(this->m_debug)
    cout << "\tfinal expand" << endl;
  if(this->Expand(newNear, cspaceDir, _new, this->m_maxDist, _lp,
      env->GetPositionRes(), env->GetOrientationRes())) {
    _lp.m_intermediates.push_back(innerCfg);
    return true;
  }
  else {
    _new = innerCfg;
    return true;
  }
}

/*----------------------------------------------------------------------------*/

#endif
