#include "BoundaryConstraint.h"

#include <limits>
#include <sstream>

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/Boundary.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"


/*------------------------------ Construction --------------------------------*/

BoundaryConstraint::
BoundaryConstraint(Robot* const _r, std::unique_ptr<Boundary>&& _b)
  : Constraint(_r), m_boundary(std::move(_b))
{ }


BoundaryConstraint::
BoundaryConstraint(Robot* const _r, XMLNode& _node)
  : Constraint(_r), m_boundary(Boundary::Factory(_node))
{ }


BoundaryConstraint::
BoundaryConstraint(const BoundaryConstraint& _other)
  : BoundaryConstraint(_other.m_robot, _other.m_boundary->Clone())
{ }


BoundaryConstraint::
~BoundaryConstraint() = default;


std::unique_ptr<Constraint>
BoundaryConstraint::
Clone() const {
  return std::unique_ptr<BoundaryConstraint>(new BoundaryConstraint(*this));
}

/*-------------------------- Constraint Interface ----------------------------*/

const Boundary*
BoundaryConstraint::
GetBoundary() const {
  return m_boundary.get();
}


bool
BoundaryConstraint::
Satisfied(const Cfg& _c) const {
  // If we require a specific robot, check that first.
  if(m_robot and m_robot != _c.GetRobot())
    return false;
  return m_boundary->InBoundary(_c);
}

/*-------------------------------- Operators ---------------------------------*/

bool
BoundaryConstraint::
operator==(const BoundaryConstraint& _b) const {
  if(m_robot != _b.m_robot)
    return false;

  auto b1 = m_boundary.get();
  auto b2 = _b.m_boundary.get();

  if(b1->Type() != b2->Type())
    return false;

  if(b1->Name() != b2->Name())
    return false;

  if(b1->GetDimension() != b2->GetDimension())
    return false;

  if(b1->GetCenter() != b2->GetCenter())
    return false;

  for(size_t i = 0; i < b1->GetDimension(); i++) {
    if(!(b1->GetRange(i) == b2->GetRange(i)))
      return false;
  }

  return true;
}
/*-------------------------------- Debugging ---------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const BoundaryConstraint& _c) {
  return _os << "BoundaryConstraint: " << *_c.GetBoundary();
}

/*----------------------------------------------------------------------------*/
