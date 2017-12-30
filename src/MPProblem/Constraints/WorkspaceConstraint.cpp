#include "WorkspaceConstraint.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/Range.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include "Transformation.h"


/*--------------------------------- Construction -----------------------------*/

WorkspaceConstraint::
WorkspaceConstraint(Robot* const _r, XMLNode& _node) : Constraint(_r) {
  throw RunTimeException(WHERE, "Not yet implemented");

  // Parse the label for the part we need to constrain.
  const std::string partLabel = _node.Read("part", true, "",
      "The label for the robot part to constrain.");
  SetPart(partLabel);

  // @TODO Parse the constraint data.
}


WorkspaceConstraint::
~WorkspaceConstraint() = default;


std::unique_ptr<Constraint>
WorkspaceConstraint::
Clone() const {
  return std::unique_ptr<WorkspaceConstraint>(new WorkspaceConstraint(*this));
}

/*------------------------------ Constraint Interface ------------------------*/

void
WorkspaceConstraint::
SetRobot(Robot* const _r) {
  // Update the body pointer.
  const std::string partLabel = m_body->Label();
  Constraint::SetRobot(_r);
  SetPart(partLabel);
}


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
  m_robot->GetMultiBody()->Configure(_c);
  const auto& currentTransform = m_body->GetWorldTransformation();

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
  auto mb = m_robot->GetMultiBody();

  // Check that the requested DOF index is valid.
  const bool outOfRange = _i >= mb->DOF();
  if(outOfRange)
    throw RunTimeException(WHERE, "Can't set a positional bound on index " +
        std::to_string(_i) + " for a multibody that has only " +
        std::to_string(mb->PosDOF()) + " positional DOFs.");

  m_constraints.push_back(
      [_i, _min, _max](const Transformation& _t) {
        const auto& position = _t.translation();
        return InRange(position[_i], _min, _max);
      });
}

/*--------------------------------- Helpers ----------------------------------*/

void
WorkspaceConstraint::
SetPart(const std::string& _label) {
  m_body = nullptr;
  auto mb = m_robot->GetMultiBody();

  for(size_t i = 0; i < mb->GetNumBodies(); ++i) {
    auto body = mb->GetBody(i);
    if(body->Label() == _label) {
      m_body = body;
      break;
    }
  }

  // Make sure we found the constrained part.
  if(!m_body)
    throw ParseException(WHERE, "Could not find robot part with label '" +
        _label + "'.");
}

/*----------------------------------------------------------------------------*/
