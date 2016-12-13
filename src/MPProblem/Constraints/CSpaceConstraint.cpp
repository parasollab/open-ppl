#include "CSpaceConstraint.h"

#include <limits>

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"


/*------------------------------ Construction --------------------------------*/

CSpaceConstraint::
CSpaceConstraint(ActiveMultiBody* const _m, XMLNode& _node) : Constraint(_m) {
  // Parse child nodes to set limits.
  for(auto& child : _node) {
    if(child.Name() == "Limit") {
      const size_t dof = child.Read("dof", true, size_t(0), size_t(0),
          size_t(_m->DOF()), "The DOF index to restrict");
      const double min = child.Read("min", true, 0.,
          std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
          "Min value");
      const double max = child.Read("max", true, 0.,
          std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
          "Max value");
      SetLimit(dof, min, max);
    }
  }
}

/*-------------------------- Constraint Interface ----------------------------*/

Boundary*
CSpaceConstraint::
GetBoundary() const {
  /// @TODO
  throw RunTimeException(WHERE, "Not yet implemented.");
  return nullptr;
}


bool
CSpaceConstraint::
Satisfied(const Cfg& _c) const {
  const auto& cfg = _c.GetData();

  // If any of the limits are violated, the constraint is not satisfied.
  for(const auto& constraint : m_limits) {
    const auto& index = constraint.first;
    const auto& range = constraint.second;

    if(!range.Inside(cfg[index]))
      return false;
  }
  return true;
}

/*--------------------------- Creation Interface -----------------------------*/

void
CSpaceConstraint::
SetLimit(const size_t _dof, const double _min, const double _max) {
  // Check that the requested DOF index is valid.
  const bool outOfRange = _dof >= m_multibody->DOF();
  if(outOfRange)
    throw RunTimeException(WHERE, "Requested limit on DOF index " +
        std::to_string(_dof) + ", but multibody has only " +
        std::to_string(m_multibody->DOF()) + " DOFs.");

  // Set the limit.
  m_limits[_dof] = Range<double>(_min, _max);
}

/*----------------------------------------------------------------------------*/
