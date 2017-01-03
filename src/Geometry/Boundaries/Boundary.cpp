#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Boundaries/Boundary.h"
#include "ConfigurationSpace/Cfg.h"


/*--------------------------- Property Accessors -----------------------------*/

const Point3d&
Boundary::
GetCenter() const noexcept {
  return m_center;
}

/*--------------------------- Containment Testing ----------------------------*/

bool
Boundary::
InBoundary(const Cfg& _cfg) const {
  auto robot = _cfg.GetRobot();

  // If the robot's center is more than a bounding radius away from the wall, it
  // is definitely inside.
  if(GetClearance(_cfg.GetRobotCenterPosition()) >=
      robot->GetBoundingSphereRadius())
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
InCSpace(const Cfg& _cfg) const {
  throw RunTimeException(WHERE, "No base class implementation supported.");
  return false;
}

/*-------------------------- CGAL Representations ----------------------------*/

Boundary::CGALPolyhedron
Boundary::
CGAL() const {
  throw RunTimeException(WHERE, "No base class implementation supported.");
  return CGALPolyhedron();
}

/*------------------------------- Display ------------------------------------*/

ostream&
operator<<(ostream& _os, const Boundary& _b) {
  _b.Write(_os);
  return _os;
}

/*----------------------------------------------------------------------------*/
