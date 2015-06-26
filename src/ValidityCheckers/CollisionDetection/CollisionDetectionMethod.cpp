#include "CollisionDetectionMethod.h"

#include "MPProblem/Geometry/Body.h"
#include "Utilities/PMPLExceptions.h"

CollisionDetectionMethod::
CollisionDetectionMethod(const string& _name, CDType _type) :
  m_name(_name), m_type(_type) {
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
IsInsideObstacle(const Vector3d& _pt, shared_ptr<Body> _body) {
  throw RunTimeException(WHERE, "Not implemented.");
}

