#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Boundaries/Boundary.h"
#include "MPProblem/ConfigurationSpace/Cfg.h"


/*------------------------------- InBoundary ---------------------------------*/

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
    const Transformation& worldTransformation = body->WorldTransformation();

    //first check just the boundary of the polyhedron
    const GMSPolyhedron& bbPoly = body->GetBoundingBoxPolyhedron();
    bool bcheck = true;
    for(const auto& v : bbPoly.m_vertexList) {
      if(!InBoundary(worldTransformation * v)) {
        bcheck = false;
        break;
      }
    }

    //boundary of polyhedron is inside the boundary thus the whole geometry is
    if(bcheck)
      continue;

    //the boundary intersected. Now check the geometry itself.
    const GMSPolyhedron& poly = body->GetPolyhedron();
    for(const auto& v : poly.m_vertexList)
      if(!InBoundary(worldTransformation * v))
        return false;
  }

  // None of the robot parts touched the boundary, so we are inside.
  return true;
}

/*------------------------------- Display ------------------------------------*/

ostream&
operator<<(ostream& _os, const Boundary& _b) {
  _b.Write(_os);
  return _os;
}

/*----------------------------------------------------------------------------*/
