#include "WorkspaceBoundingSphere.h"


/*------------------------------ Construction --------------------------------*/

WorkspaceBoundingSphere::
WorkspaceBoundingSphere(const size_t _n, const double _radius)
  : AbstractBoundingSphere(_n, _radius) {
  if(_n != 2 and _n != 3)
    throw RunTimeException(WHERE, "Workspace boundaries must have 2 or 3 "
        "dimensions.");
}


WorkspaceBoundingSphere::
WorkspaceBoundingSphere(const std::vector<double>& _center, const double _radius)
  : AbstractBoundingSphere(_center, _radius) {
  if(_center.size() != 2 and _center.size() != 3)
    throw RunTimeException(WHERE, "Workspace boundaries must have 2 or 3 "
        "dimensions.");
}


WorkspaceBoundingSphere::
WorkspaceBoundingSphere(const Vector3d& _center, const double _radius)
  : AbstractBoundingSphere({_center[0], _center[1], _center[2]}, _radius) { }


std::unique_ptr<Boundary>
WorkspaceBoundingSphere::
Clone() const {
  return std::unique_ptr<Boundary>(new WorkspaceBoundingSphere(*this));
}

/*---------------------------- Property Accessors ----------------------------*/

Boundary::Space
WorkspaceBoundingSphere::
Type() const noexcept {
  return Boundary::Space::Workspace;
}


std::string
WorkspaceBoundingSphere::
Name() const noexcept {
  return "WorkspaceBoundingSphere";
}

/*--------------------------- Containment Testing ----------------------------*/

bool
WorkspaceBoundingSphere::
InBoundary(const Cfg& _cfg) const {
  return InWorkspace(_cfg);
}

/*----------------------------------------------------------------------------*/
