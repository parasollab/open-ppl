#include "CollisionDetectionMethod.h"

CollisionDetectionMethod::CollisionDetectionMethod(string _name, CDType _type, cd_predefined _cdtype) : 
  m_name(_name), m_type(_type), m_cdtype(_cdtype) {
  }

CollisionDetectionMethod::~CollisionDetectionMethod() {}

bool
CollisionDetectionMethod::operator==(const CollisionDetectionMethod& _cd) const {
  return m_name == _cd.m_name;
}

void
CollisionDetectionMethod::PrintOptions(ostream& _os) const {
  _os << "\t" << m_name << " " << endl;
}

bool 
CollisionDetectionMethod::IsInsideObstacle(const Cfg& _cfg, Environment* _env, CDInfo& _cdInfo) {
  cerr << "IsInsideObstacle: Not implemeneted yet" << endl;
  exit(1);
}

