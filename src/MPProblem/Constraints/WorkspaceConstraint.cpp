#include "WorkspaceConstraint.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Bodies/FreeBody.h"
#include "Geometry/Boundaries/Range.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"

#include "Transformation.h"


/*--------------------------------- Construction -----------------------------*/

WorkspaceConstraint::
WorkspaceConstraint(ActiveMultiBody* const _m, XMLNode& _node) : Constraint(_m) {
  throw RunTimeException(WHERE, "Not yet implemented");

  // Parse the label for the part we need to constrain.
  std::string partLabel = _node.Read("part", true, "", "The label for the robot "
      "part to constrain.");

  // Find the part.
  for(size_t i = 0; i < m_multibody->NumFreeBody(); ++i) {
    auto body = m_multibody->GetFreeBody(i);
    if(body->Label() == partLabel) {
      m_freeBody = body;
      break;
    }
  }

  // Make sure we found it.
  if(!m_freeBody)
    throw ParseException(WHERE, "Could not find robot part with label '" +
        partLabel + "'.");

  // @TODO Parse the constraint data.
}

/*------------------------------ Constraint Interface ------------------------*/

const Boundary*
WorkspaceConstraint::
GetBoundary() const {
  /// @TODO
  throw RunTimeException(WHERE, "Not yet implemented");
  return nullptr;
}


bool
WorkspaceConstraint::
Satisfied(const Cfg& _c) const {
  // Configure the object at _c and get the transformation for the constrained
  // free body.
  m_multibody->Configure(_c);
  const auto& currentTransform = m_freeBody->GetWorldTransformation();

  // Check the current transform against each constraint function. If any fail,
  // then the constraint isn't satisfied.
  for(auto& constraintFunction : m_constraints)
    if(!constraintFunction(currentTransform))
      return false;
  return true;
}

/*------------------------------- Creation Interface -------------------------*/

void
WorkspaceConstraint::
AddFunction(ConstraintFunction&& _f) {
  m_constraints.push_back(_f);
}


void
WorkspaceConstraint::
SetPositionalBound(const size_t _i, const double _min, const double _max) {
  // Check that the requested DOF index is valid.
  const bool outOfRange = _i >= m_multibody->DOF();
  if(outOfRange)
    throw RunTimeException(WHERE, "Can't set a positional bound on index " +
        std::to_string(_i) + " for a multibody that has only " +
        std::to_string(m_multibody->PosDOF()) + " positional DOFs.");

  m_constraints.push_back(
      [_i, _min, _max](const Transformation& _t) {
        const auto& position = _t.translation();
        return InRange(position[_i], _min, _max);
      });
}

/*----------------------------------------------------------------------------*/
