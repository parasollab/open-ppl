#include "CollisionDetectionMethod.h"
#include "Utilities/PMPLExceptions.h"

#include <iostream>

/*------------------------------- Construction -------------------------------*/

CollisionDetectionMethod::
CollisionDetectionMethod(const string& _name) : m_name(_name) { }

/*--------------------------------- Accessors --------------------------------*/

void
CollisionDetectionMethod::
Print(ostream& _os) const {
  _os << m_name << endl;
}

/*------------------------------- CD Interface -------------------------------*/

bool
CollisionDetectionMethod::
IsInsideObstacle(const Vector3d& _pt, shared_ptr<Body> _body) {
  throw RunTimeException(WHERE, "Not implemented.");
}

/*----------------------------------------------------------------------------*/
