#include "TransformConstraint.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Bodies/FreeBody.h"
#include "Utilities/RuntimeUtils.h"

#include "Transformation.h"


/*--------------------------------- Construction -----------------------------*/

TransformConstraint::
TransformConstraint(ActiveMultiBody* _m, FreeBody* _f) : Constraint(_m),
    m_freeBody(_f) { }

/*------------------------------ Constraint Interface ------------------------*/

const bool
TransformConstraint::
operator()(const Cfg& _c) const {
  // Configure the object at _c and get its transformation.
  m_multibody->Configure(_c);
  const auto& currentTransform = m_freeBody->GetWorldTransformation();

  // Check the current transform against each constraint function. If any fail,
  // then the constraint isn't satisfied.
  for(auto& constraint : m_constraints)
    if(!constraint(currentTransform))
      return false;
  return true;
}

/*------------------------------- Creation Interface -------------------------*/

void
TransformConstraint::
SetPositionalBound(const size_t _i, const double _min, const double _max) {
  AssertMsg(_i < m_multibody->PosDOF(), WHERE + "\nCan't set a positional bound "
      "on index " + std::to_string(_i) + " for a multibody that has only " +
      std::to_string(m_multibody->PosDOF()) + " positional DOFs.");

  m_constraints.push_back(
      [_i, _min, _max](const Transformation& _t) {
        const auto& position = _t.translation();
        return Range<double>(_min, _max).Inside(position[_i]);
      });
}

/*----------------------------------------------------------------------------*/
