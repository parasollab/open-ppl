#include "CenterOfMassDistance.h"

CenterOfMassDistance::CenterOfMassDistance() : DistanceMetricMethod() {
  m_name = "com";
}

CenterOfMassDistance::CenterOfMassDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : 
  DistanceMetricMethod(_node, _problem, _warn) {
    m_name = "com";
  }

CenterOfMassDistance::~CenterOfMassDistance() {}

//Calculate distance
double CenterOfMassDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  return Distance(_c1, _c2);
}

//Calculate the center of mass from the distance above
double CenterOfMassDistance::Distance(const Cfg& _c1, const Cfg& _c2) {
  return (_c1.GetRobotCenterPosition() - _c2.GetRobotCenterPosition()).magnitude();
}
