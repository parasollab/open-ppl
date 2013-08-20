#include "BinaryLPSweptDistance.h"
#include "LocalPlanners.h"


BinaryLPSweptDistance::BinaryLPSweptDistance() : LPSweptDistance() {
  m_name = "binary_lp_swept";
}

BinaryLPSweptDistance::
BinaryLPSweptDistance(string _lp, double _posRes, double _oriRes, double _tolerance, int _maxAttempts, bool _bbox) :
  LPSweptDistance(_lp, _posRes, _oriRes, _bbox),
  m_tolerance(_tolerance), m_maxAttempts(_maxAttempts) {
  m_name = "binary_lp_swept";
}

BinaryLPSweptDistance::
BinaryLPSweptDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : LPSweptDistance(_node, _problem, false) {
  m_name = "binary_lp_swept";
  m_tolerance = _node.numberXMLParameter("tolerance", false, 0.01, 0.0, 1000.0, "tolerance");
  m_maxAttempts = _node.numberXMLParameter("max_attempts", false, 10, 1, 100, "maximum depth of lp_swept distance search");

  if(_warn)
    _node.warnUnrequestedAttributes();
}

BinaryLPSweptDistance::~BinaryLPSweptDistance() {}

void BinaryLPSweptDistance::PrintOptions(ostream& _os) const {
  LPSweptDistance::PrintOptions(_os);
  _os << "\ttolerance = " << m_tolerance;
  _os << "\tmax_attempts = " << m_maxAttempts;
}


double BinaryLPSweptDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  double positionResSave = m_positionRes;
  double orientationResSave = m_orientationRes;

  double oldDist = LPSweptDistance::Distance(_env, _c1, _c2);
  double newDist = 0.0;
  int matchCount = 1;

  for (int i = 1; i < m_maxAttempts; i++) {
    m_positionRes = max(m_positionRes/2.0, _env->GetPositionRes());
    m_orientationRes = max(m_orientationRes/2.0, _env->GetOrientationRes());

    newDist = LPSweptDistance::Distance(_env, _c1, _c2);

    if (newDist - oldDist < m_tolerance)
      matchCount++;
    else
      matchCount = 1;

    if (matchCount == 3)
      break;

    if (m_positionRes == _env->GetPositionRes() && m_orientationRes == _env->GetOrientationRes())
      break;

    oldDist = newDist;
  }

  m_positionRes = positionResSave;
  m_orientationRes = orientationResSave;

  return newDist;
}

