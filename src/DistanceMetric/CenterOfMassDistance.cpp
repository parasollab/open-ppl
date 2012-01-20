#include "CenterOfMassDistance.h"

CenterOfMassDistance::CenterOfMassDistance() : DistanceMetricMethod() {
  m_name = "com";
}

CenterOfMassDistance::  
CenterOfMassDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : DistanceMetricMethod(_node, _problem, _warn) {
  m_name = "com";
}

CenterOfMassDistance::~CenterOfMassDistance() {}

void CenterOfMassDistance::PrintOptions(ostream& _os) const {
  _os << "    " << GetName() << "::  " << endl;
}

double CenterOfMassDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2){
  return Distance(_c1, _c2);
}
    
double CenterOfMassDistance::Distance(const Cfg& _c1, const Cfg& _c2) {
  Vector3D d = _c1.GetRobotCenterPosition()-_c2.GetRobotCenterPosition();
  return d.magnitude();
}
