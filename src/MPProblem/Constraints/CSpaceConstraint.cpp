#include "CSpaceConstraint.h"

#include <limits>
#include <sstream>

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"


/*------------------------------ Construction --------------------------------*/
CSpaceConstraint::
CSpaceConstraint(Robot* const _r, const Cfg& _cfg)
  : Constraint(_r),
    m_bbx(_r->GetMultiBody()->DOF() * (_r->IsNonholonomic() ? 2 : 1))
{
    // This is a point constraint.
    Cfg point(_r);
    point = _cfg;

    m_bbx.ShrinkToPoint(point);
}

CSpaceConstraint::
CSpaceConstraint(Robot* const _r, const std::string& _pointString)
  : Constraint(_r),
    m_bbx(_r->GetMultiBody()->DOF() * (_r->IsNonholonomic() ? 2 : 1))
{
    // This is a point constraint.
    Cfg point(_r);
    std::istringstream pointStream(_pointString);
    point.Read(pointStream);

    m_bbx.ShrinkToPoint(point);
}

CSpaceConstraint::
CSpaceConstraint(Robot* const _r, XMLNode& _node)
  : Constraint(_r),
    m_bbx(_r->GetMultiBody()->DOF() * (_r->IsNonholonomic() ? 2 : 1))
{
  // The XML node will either describe a point or a bounding box.
  const std::string pointString = _node.Read("point", false, "", "The Cfg point"),
                    bbxString = _node.Read("bbx", false, "", "The Cfg box");

  // Assert that we got exactly one of these.
  if(pointString.empty() == bbxString.empty())
    throw ParseException(_node.Where(), "A CSpaceConstraint should specify a "
        "single configuration (point) or bounding box (bbx), but not both.");

  // Parse the boundary data.
  if(!bbxString.empty()) {
    // This is a bounding box constraint.
    std::istringstream bbxStream(bbxString);
    bbxStream >> m_bbx;
  }
  else {
    // This is a point constraint.
    Cfg point(_r);
    std::istringstream pointStream(pointString);
    point.Read(pointStream);

    m_bbx.ShrinkToPoint(point);
  }
}

/*-------------------------- Constraint Interface ----------------------------*/

const Boundary*
CSpaceConstraint::
GetBoundary() const {
  return &m_bbx;
}


bool
CSpaceConstraint::
Satisfied(const Cfg& _c) const {
  return m_bbx.InBoundary(_c);
}

/*--------------------------- Creation Interface -----------------------------*/

void
CSpaceConstraint::
SetLimit(const size_t _dof, const double _min, const double _max) {
  // Check that the requested DOF index is valid.
  const bool outOfRange = _dof >= m_robot->GetMultiBody()->DOF();
  if(outOfRange)
    throw RunTimeException(WHERE, "Requested limit on DOF index " +
        std::to_string(_dof) + ", but multibody has only " +
        std::to_string(m_robot->GetMultiBody()->DOF()) + " DOFs.");

  // Set the limit.
  m_bbx.SetRange(_dof, _min, _max);
}

/*-------------------------------- Debugging ---------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const CSpaceConstraint& _c) {
  return _os << static_cast<const NBox&>(_c.m_bbx);
}

/*----------------------------------------------------------------------------*/
