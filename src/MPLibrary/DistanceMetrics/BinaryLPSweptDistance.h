#ifndef PMPL_BINARY_LP_SWEPT_DISTANCE_H_
#define PMPL_BINARY_LP_SWEPT_DISTANCE_H_

#include "LPSweptDistance.h"


////////////////////////////////////////////////////////////////////////////////
/// @todo This class should be replaced by an option in the base class.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class BinaryLPSweptDistance : public LPSweptDistance<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    BinaryLPSweptDistance(string _lp = "", double _posRes = .1,
        double _oriRes = .1, double _tolerance = .01, int _maxAttempts = 100,
        bool _bbox = false);
    BinaryLPSweptDistance(XMLNode& _node);

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    double m_tolerance;
    int m_maxAttempts;

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
BinaryLPSweptDistance<MPTraits>::
BinaryLPSweptDistance(string _lp, double _posRes, double _oriRes,
    double _tolerance, int _maxAttempts, bool _bbox) :
    LPSweptDistance<MPTraits>(_lp, _posRes, _oriRes, _bbox),
  m_tolerance(_tolerance), m_maxAttempts(_maxAttempts) {
  this->SetName("BinaryLPSwept");
}


template <typename MPTraits>
BinaryLPSweptDistance<MPTraits>::
BinaryLPSweptDistance(XMLNode& _node) : LPSweptDistance<MPTraits>(_node) {
  this->SetName("BinaryLPSwept");
  m_tolerance = _node.Read("tolerance", false, 0.01, 0.0, 1000.0, "tolerance");
  m_maxAttempts = _node.Read("maxAttempts", false, 10, 1, 100, "maximum depth "
      "of lp_swept distance search");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
BinaryLPSweptDistance<MPTraits>::
Print(ostream& _os) const {
  LPSweptDistance<MPTraits>::Print(_os);
  _os << "\ttolerance = " << m_tolerance << endl;
  _os << "\tmaxAttempts = " << m_maxAttempts << endl;
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
BinaryLPSweptDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  double positionResSave = this->m_positionRes;
  double orientationResSave = this->m_orientationRes;

  Environment* env = this->GetEnvironment();
  double oldDist = LPSweptDistance<MPTraits>::Distance(_c1, _c2);
  double newDist = 0.0;
  int matchCount = 1;

  for(int i = 1; i < m_maxAttempts; i++) {
    this->m_positionRes = max(this->m_positionRes / 2., env->GetPositionRes());
    this->m_orientationRes = max(this->m_orientationRes / 2.,
        env->GetOrientationRes());

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

/*----------------------------------------------------------------------------*/

#endif
