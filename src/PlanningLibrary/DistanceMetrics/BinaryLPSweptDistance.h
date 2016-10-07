#ifndef BINARY_LP_SWEPT_DISTANCE_H_
#define BINARY_LP_SWEPT_DISTANCE_H_

#include "LPSweptDistance.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class BinaryLPSweptDistance : public LPSweptDistance<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType CfgType;

    BinaryLPSweptDistance(string _lp = "", double _posRes = 0.1, double _oriRes = 0.1,
        double _tolerance = 0.01, int _maxAttempts = 100, bool _bbox = false);
    BinaryLPSweptDistance(MPProblemType* _problem, XMLNode& _node);

    virtual void Print(ostream& _os) const;

    virtual double Distance(const CfgType& _c1, const CfgType& _c2);

  private:
    double m_tolerance;
    int m_maxAttempts;
};

template<class MPTraits>
BinaryLPSweptDistance<MPTraits>::
BinaryLPSweptDistance(string _lp, double _posRes, double _oriRes,
    double _tolerance, int _maxAttempts, bool _bbox) :
  LPSweptDistance<MPTraits>(_lp, _posRes, _oriRes, _bbox),
  m_tolerance(_tolerance), m_maxAttempts(_maxAttempts) {
    this->SetName("BinaryLPSwept");
  }

template<class MPTraits>
BinaryLPSweptDistance<MPTraits>::
BinaryLPSweptDistance(MPProblemType* _problem, XMLNode& _node) :
  LPSweptDistance<MPTraits>(_problem, _node) {
    this->SetName("BinaryLPSwept");
    m_tolerance = _node.Read("tolerance", false, 0.01, 0.0, 1000.0, "tolerance");
    m_maxAttempts = _node.Read("maxAttempts", false, 10, 1, 100, "maximum depth of lp_swept distance search");
  }

template<class MPTraits>
void
BinaryLPSweptDistance<MPTraits>::
Print(ostream& _os) const {
  LPSweptDistance<MPTraits>::Print(_os);
  _os << "\ttolerance = " << m_tolerance << endl;
  _os << "\tmaxAttempts = " << m_maxAttempts << endl;
}

template<class MPTraits>
double
BinaryLPSweptDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  double positionResSave = this->m_positionRes;
  double orientationResSave = this->m_orientationRes;

  Environment* env = this->GetMPProblem()->GetEnvironment();
  double oldDist = LPSweptDistance<MPTraits>::Distance(_c1, _c2);
  double newDist = 0.0;
  int matchCount = 1;

  for(int i = 1; i < m_maxAttempts; i++) {
    this->m_positionRes = max(this->m_positionRes/2.0, env->GetPositionRes());
    this->m_orientationRes = max(this->m_orientationRes/2.0, env->GetOrientationRes());

    newDist = LPSweptDistance<MPTraits>::Distance(_c1, _c2);

    if(newDist - oldDist < m_tolerance)
      matchCount++;
    else
      matchCount = 1;

    if(matchCount == 3)
      break;

    if(this->m_positionRes == env->GetPositionRes() &&
        this->m_orientationRes == env->GetOrientationRes())
      break;

    oldDist = newDist;
  }

  this->m_positionRes = positionResSave;
  this->m_orientationRes = orientationResSave;

  return newDist;
}

#endif
