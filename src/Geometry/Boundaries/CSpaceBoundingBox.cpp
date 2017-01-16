#include "CSpaceBoundingBox.h"


/*------------------------------ Construction --------------------------------*/

CSpaceBoundingBox::
CSpaceBoundingBox(const size_t _n) : AbstractBoundingBox(_n) { }


CSpaceBoundingBox::
CSpaceBoundingBox(const std::vector<double>& _center) :
    AbstractBoundingBox(_center) { }


Boundary*
CSpaceBoundingBox::
Clone() const {
  return new CSpaceBoundingBox(*this);
}

/*--------------------------- Property Accessors -----------------------------*/

std::string
CSpaceBoundingBox::
Type() const noexcept {
  return "CSpaceBoundingBox";
}

/*--------------------------- Containment Testing ----------------------------*/

bool
CSpaceBoundingBox::
InBoundary(const Cfg& _c) const {
  return Boundary::InCSpace(_c);
}

/*----------------------------------------------------------------------------*/
