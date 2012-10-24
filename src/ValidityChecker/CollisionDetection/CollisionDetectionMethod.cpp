#include "CollisionDetectionMethod.h"

CollisionDetectionMethod::
CollisionDetectionMethod() {
  m_cdtype = CD_USER1;
}

CollisionDetectionMethod::
~CollisionDetectionMethod() {
}

bool
CollisionDetectionMethod::
operator==(const CollisionDetectionMethod& _cd) const {
  return GetName() == _cd.GetName();
}

int
CollisionDetectionMethod::
GetType() {
  return m_type;
}

void
CollisionDetectionMethod::
PrintOptions(ostream& _os) const {
  _os << "    " << GetName() << " ";
  _os << endl;
}

bool 
CollisionDetectionMethod::
IsInsideObstacle(const Cfg& _cfg, Environment* _env, CDInfo& _cdInfo) {
  cerr<<"IsInsideObstacle: Not implemeneted yet"<<endl;
  exit(1);
  return false;
}

