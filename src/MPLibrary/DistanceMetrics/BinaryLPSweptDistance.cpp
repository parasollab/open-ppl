#include "BinaryLPSweptDistance.h"


/*------------------------------- Construction -------------------------------*/

BinaryLPSweptDistance::
BinaryLPSweptDistance(string _lp, double _posRes, double _oriRes,
    double _tolerance, int _maxAttempts, bool _bbox) :
    LPSweptDistance(_lp, _posRes, _oriRes, _bbox),
  m_tolerance(_tolerance), m_maxAttempts(_maxAttempts) {
  this->SetName("BinaryLPSwept");
}


BinaryLPSweptDistance::
BinaryLPSweptDistance(XMLNode& _node) : DistanceMetricMethod(_node),
                                        LPSweptDistance(_node) {
  this->SetName("BinaryLPSwept");
  m_tolerance = _node.Read("tolerance", false, 0.01, 0.0, 1000.0, "tolerance");
  m_maxAttempts = _node.Read("maxAttempts", false, 10, 1, 100, "maximum depth "
      "of lp_swept distance search");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

void
BinaryLPSweptDistance::
Print(ostream& _os) const {
  LPSweptDistance::Print(_os);
  _os << "\ttolerance = " << m_tolerance << endl;
  _os << "\tmaxAttempts = " << m_maxAttempts << endl;
}

/*----------------------------- Distance Interface ---------------------------*/

double
BinaryLPSweptDistance::
Distance(const Cfg& _c1, const Cfg& _c2) {
  // save old resolutions
  double positionResSave = this->m_positionRes;
  double orientationResSave = this->m_orientationRes;

  Environment* env = this->GetEnvironment();
  // init distance to LPSweptDistance between cfg's with starting resolutions
  double oldDist = LPSweptDistance::Distance(_c1, _c2);
  double newDist = 0.0;
  int matchCount = 1;

  // until maxAttempts reached, distance converges, or resolution becomes more
  // granular than environmental resolution
  for(int i = 1; i < m_maxAttempts; i++) {
    // attempt to divide resolutions by 2 (or use env resolution if too small)
    this->m_positionRes = max(this->m_positionRes / 2., env->GetPositionRes());
    this->m_orientationRes = max(this->m_orientationRes / 2.,
        env->GetOrientationRes());
    // recalculate distance with new resolutions
    newDist = LPSweptDistance::Distance(_c1, _c2);

    // if distance is within tolerance 3 times in a row, consider distance as
    // converged and break
    if(newDist - oldDist < m_tolerance)
      matchCount++;
    else
      matchCount = 1;

    if(matchCount == 3)
      break;

    // if resolutions become equal to environemntal resolutions, break
    if(this->m_positionRes == env->GetPositionRes() &&
        this->m_orientationRes == env->GetOrientationRes())
      break;

    oldDist = newDist;
  }
  // restore original resolutions
  this->m_positionRes = positionResSave;
  this->m_orientationRes = orientationResSave;

  return newDist;
}

