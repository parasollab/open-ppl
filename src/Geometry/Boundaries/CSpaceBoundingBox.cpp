#include "CSpaceBoundingBox.h"

#include "ConfigurationSpace/Cfg.h"


/*------------------------------ Construction --------------------------------*/

CSpaceBoundingBox::
CSpaceBoundingBox(const size_t _n) : AbstractBoundingBox(_n) { }


CSpaceBoundingBox::
CSpaceBoundingBox(const std::vector<double>& _center) :
    AbstractBoundingBox(_center) { }


std::unique_ptr<Boundary>
CSpaceBoundingBox::
Clone() const {
  return std::unique_ptr<Boundary>(new CSpaceBoundingBox(*this));
}

/*--------------------------- Property Accessors -----------------------------*/

Boundary::Space
CSpaceBoundingBox::
Type() const noexcept {
  return Boundary::Space::CSpace;
}


std::string
CSpaceBoundingBox::
Name() const noexcept {
  return "CSpaceBoundingBox";
}

/*--------------------------- Containment Testing ----------------------------*/

bool
CSpaceBoundingBox::
InBoundary(const Cfg& _c) const {
  return Boundary::InCSpace(_c);
}

/*--------------------------- Special Modifiers ------------------------------*/

void
CSpaceBoundingBox::
ShrinkToPoint(const Cfg& _c) noexcept {
  const size_t dof = _c.DOF();

  for(size_t i = 0; i < dof; ++i)
    NBox::SetRange(i, _c[i], _c[i]);

  if(_c.IsNonholonomic())
    for(size_t i = 0; i < dof; ++i)
      NBox::SetRange(i + dof, _c.Velocity(i), _c.Velocity(i));
}

/*----------------------------------------------------------------------------*/
