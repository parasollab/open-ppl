#include "CollisionDetectionMethod.h"
#include "Utilities/PMPLExceptions.h"


/*------------------------------- Construction -------------------------------*/

CollisionDetectionMethod::
CollisionDetectionMethod(const std::string& _name) : m_name(_name) { }


CollisionDetectionMethod::
~CollisionDetectionMethod() = default;

/*--------------------------------- Accessors --------------------------------*/

const std::string&
CollisionDetectionMethod::
GetName() const {
  return m_name;
}


void
CollisionDetectionMethod::
Print(std::ostream& _os) const {
  _os << m_name << std::endl;
}

/*------------------------------- CD Interface -------------------------------*/

bool
CollisionDetectionMethod::
IsInsideObstacle(const mathtool::Vector3d& _pt, const GMSPolyhedron&,
    const mathtool::Transformation&) {
  throw NotImplementedException(WHERE);
}

/*----------------------------------------------------------------------------*/
