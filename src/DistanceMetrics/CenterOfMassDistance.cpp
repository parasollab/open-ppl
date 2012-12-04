#include "CenterOfMassDistance.h"

CenterOfMassDistance::CenterOfMassDistance() : DistanceMetricMethod() {
  m_name = "com";
}

CenterOfMassDistance::CenterOfMassDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : 
  DistanceMetricMethod(_node, _problem, _warn) {
    m_name = "com";
  }

CenterOfMassDistance::~CenterOfMassDistance() {}

double CenterOfMassDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  return (_c1.GetRobotCenterofMass( _env) - _c2.GetRobotCenterofMass(_env)).magnitude();
 
}

