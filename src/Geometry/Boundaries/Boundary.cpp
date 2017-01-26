#include "Geometry/Boundaries/Boundary.h"

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/ActiveMultiBody.h"


/*--------------------------- Containment Testing ----------------------------*/

bool
Boundary::
InBoundary(const Vector3d& _p) const {
  return InBoundary(std::vector<double>{_p[0], _p[1], _p[2]});
}

/*--------------------------- Containment Helpers ----------------------------*/

bool
Boundary::
InWorkspace(const Cfg& _cfg) const {
  auto robot = _cfg.GetMultiBody();

  // If the robot's center is more than a bounding radius away from the wall, it
  // is definitely inside.
  if(GetClearance(_cfg.GetPoint()) >= robot->GetBoundingSphereRadius())
    return true;

  // Robot is close to wall, have a strict check.
  _cfg.ConfigureRobot();

  // Check each part of the robot multibody for being inside of the boundary.
  for(size_t m = 0; m < robot->NumFreeBody(); ++m) {
    const auto body = robot->GetFreeBody(m);

    // First check if the body's bounding box intersects the boundary.
    bool bbxGood = true;
    const GMSPolyhedron bbx = body->GetWorldBoundingBox();
    for(const auto& v : bbx.m_vertexList) {
      if(!InBoundary(v)) {
        bbxGood = false;
        break;
      }
    }

    // If the bounding box is inside the boundary, the whole geometry must be.
    // Skip the fine-grain check.
    if(bbxGood)
      continue;

    // If the bounding box touches the boundary, we need to check each vertex in
    // the body geometry.
    const GMSPolyhedron& poly = body->GetWorldPolyhedron();
    for(const auto& v : poly.m_vertexList)
      if(!InBoundary(v))
        return false;
  }

  // None of the robot parts touched the boundary, so we are inside.
  return true;
}


bool
Boundary::
InCSpace(const Cfg& _c) const {
  return InBoundary(_c.GetData());
}

/*----------------------- Clearance Testing ----------------------------------*/

double
Boundary::
GetClearance(const Vector3d& _p) const {
  throw RunTimeException(WHERE, "No base class implementation supported.");
  return 0.;
}


Vector3d
Boundary::
GetClearancePoint(const Vector3d& _p) const {
  throw RunTimeException(WHERE, "No base class implementation supported.");
  return Vector3d();
}

/*-------------------------- CGAL Representations ----------------------------*/

Boundary::CGALPolyhedron
Boundary::
CGAL() const {
  throw RunTimeException(WHERE, "No base class implementation supported.");
  return CGALPolyhedron();
}

/*----------------------------------- I/O ------------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const Boundary& _b) {
  /// @TODO Synchronize type printing with input parsing.
  _os << _b.Type() << " ";
  _b.Write(_os);
  return _os;
}


/// @TODO Move implementation from Environment.cpp to here.
//std::istream&
//operator>>(std::istream& _is, const Boundary& _b) {
//  return _is;
//}

/*----------------------------------------------------------------------------*/
