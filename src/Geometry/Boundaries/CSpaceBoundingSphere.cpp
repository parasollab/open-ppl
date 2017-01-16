#include "CSpaceBoundingSphere.h"


/*------------------------------ Construction --------------------------------*/

CSpaceBoundingSphere::
CSpaceBoundingSphere(const size_t _n, const double _radius)
  : AbstractBoundingSphere(_n, _radius) { }


CSpaceBoundingSphere::
CSpaceBoundingSphere(const std::vector<double>& _center, const double _radius)
  : AbstractBoundingSphere(_center, _radius) { }


Boundary*
CSpaceBoundingSphere::
Clone() const {
  return new CSpaceBoundingSphere(*this);
}

/*---------------------------- Property Accessors ----------------------------*/

std::string
CSpaceBoundingSphere::
Type() const noexcept {
  return "CSpaceBoundingSphere";
}

/*--------------------------- Containment Testing ----------------------------*/

bool
CSpaceBoundingSphere::
InBoundary(const Cfg& _cfg) const {
  return InCSpace(_cfg);
}

/*----------------------------------------------------------------------------*/
