#include "DOFConstraint.h"

#include "Utilities/RuntimeUtils.h"

/*--------------------------------- Construction -----------------------------*/

DOFConstraint::
DOFConstraint(ActiveMultiBody* _m) : Constraint(_m) { }

/*------------------------------ Constraint Interface ------------------------*/

const bool
DOFConstraint::
operator()(const Cfg& _c) const {
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

/*------------------------------- Creation Interface -------------------------*/

void
DOFConstraint::
SetLimit(const size_t _dof, const double _min, const double _max) {
  AssertMsg(_dof < m_multibody->DOF(), WHERE + "\nRequested limit on DOF "
      "index " + std::to_string(_dof) + ", but multibody has only " +
      std::to_string(m_multibody->DOF()) + " DOFs.");

  m_limits[dof] = Range<double>(_min, _max);
}

/*----------------------------------------------------------------------------*/
