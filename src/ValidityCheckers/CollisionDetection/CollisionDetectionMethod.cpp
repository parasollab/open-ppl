#include "CollisionDetectionMethod.h"

CollisionDetectionMethod::
CollisionDetectionMethod(string _name, CDType _type, cd_predefined _cdType) :
  m_name(_name), m_type(_type), m_cdType(_cdType) {
  }

CollisionDetectionMethod::
~CollisionDetectionMethod() {
}

void
CollisionDetectionMethod::
Print(ostream& _os) const {
  _os << "\t" << m_name << " " << endl;
}

bool
CollisionDetectionMethod::
IsInsideObstacle(const Cfg& _cfg) {
  throw RunTimeException(WHERE, "Not implemented.");
}

