#include "MPTask.h"

#include "Geometry/Boundaries/Boundary.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"


/*------------------------------ Construction --------------------------------*/

MPTask::
MPTask(Robot* const _robot) : m_robot(_robot) {
  m_label = "null task";
}


MPTask::
MPTask(MPProblem* const _problem, XMLNode& _node) {
  // Parse task and robot labels.
  m_label = _node.Read("label", true, "", "Unique label for this task");

  // Get the robot by label.
  const std::string robotLabel = _node.Read("robot", true, "", "Label for the "
      "robot assigned to this task.");
  m_robot = _problem->GetRobot(robotLabel);

  // Parse constraints.
  for(auto& child : _node) {
    if(child.Name() == "StartConstraints") {
      for(auto& grandChild : child)
        m_startConstraints.push_back(Constraint::Factory(m_robot, grandChild));
    }
    else if(child.Name() == "PathConstraints") {
      for(auto& grandChild : child)
        m_pathConstraints.push_back(Constraint::Factory(m_robot, grandChild));
    }
    else if(child.Name() == "GoalConstraints") {
      for(auto& grandChild : child)
        m_goalConstraints.push_back(Constraint::Factory(m_robot, grandChild));
    }
  }
}


MPTask::
MPTask(const MPTask& _other) {
  *this = _other;
}


MPTask::
MPTask(MPTask&& _other) = default;


MPTask::
~MPTask() = default;

/*-------------------------------- Assignment --------------------------------*/

MPTask&
MPTask::
operator=(const MPTask& _other) {
  if(this != &_other) {
    m_label      = _other.m_label;
    m_robot      = _other.m_robot;

    for(const auto& c : _other.m_startConstraints)
      m_startConstraints.push_back(c->Clone());
    for(const auto& c : _other.m_pathConstraints)
      m_pathConstraints.push_back(c->Clone());
    for(const auto& c : _other.m_goalConstraints)
      m_goalConstraints.push_back(c->Clone());
  }

  return *this;
}


MPTask&
MPTask::
operator=(MPTask&& _other) = default;

/*--------------------------- Property Accessors -----------------------------*/

Robot*
MPTask::
GetRobot() const noexcept {
  return m_robot;
}


void
MPTask::
SetRobot(Robot* const _r) {
  m_robot = _r;

  for(auto& c : m_startConstraints)
    c->SetRobot(_r);
  for(auto& c : m_pathConstraints)
    c->SetRobot(_r);
  for(auto& c : m_goalConstraints)
    c->SetRobot(_r);
}


const std::string&
MPTask::
GetLabel() const noexcept {
  return m_label;
}


void
MPTask::
SetLabel(const std::string& _label) noexcept {
  m_label = _label;
}

/*-------------------------- Constraint Accessors ----------------------------*/

void
MPTask::
AddStartConstraint(std::unique_ptr<Constraint>&& _c) {
  m_startConstraints.push_back(std::move(_c));
}


void
MPTask::
AddPathConstraint(std::unique_ptr<Constraint>&& _c) {
  m_pathConstraints.push_back(std::move(_c));
}


void
MPTask::
AddGoalConstraint(std::unique_ptr<Constraint>&& _c) {
  m_goalConstraints.push_back(std::move(_c));
}


const MPTask::ConstraintSet&
MPTask::
GetStartConstraints() const noexcept {
  return m_startConstraints;
}


const MPTask::ConstraintSet&
MPTask::
GetPathConstraints() const noexcept {
  return m_pathConstraints;
}


const MPTask::ConstraintSet&
MPTask::
GetGoalConstraints() const noexcept {
  return m_goalConstraints;
}


const Boundary*
MPTask::
GetStartBoundary() const noexcept {
  return MakeComposeBoundary(m_startConstraints);
}


const Boundary*
MPTask::
GetPathBoundary() const noexcept {
  return MakeComposeBoundary(m_pathConstraints);
}


const Boundary*
MPTask::
GetGoalBoundary() const noexcept {
  return MakeComposeBoundary(m_goalConstraints);
}

/*------------------------------- Task Status --------------------------------*/

bool
MPTask::
IsCompleted() const {
  return m_status == Complete;
}


void
MPTask::
SetCompleted() {
  m_status = Complete;
}


bool
MPTask::
IsStarted() const {
  return m_status == InProgress or IsCompleted();
}


void
MPTask::
SetStarted() {
  if(!IsStarted())
    m_status = InProgress;
}


void
MPTask::
Reset() {
  m_status = OnDeck;
}

/*---------------------------- Constraint Evaluation -------------------------*/

MPTask::Status
MPTask::
Evaluate(const std::vector<Cfg>& _p) const {
  // If start constraints are not satisfied, this task is still on deck.
  if(m_status == OnDeck and !EvaluateStartConstraints(_p))
    return OnDeck;

  // If path constraints aren't satisfied, this is an invalid path.
  if(!EvaluatePathConstraints(_p))
    return Invalid;

  // If goal constraints are satisfied, task as finished. Otherwise, it's in
  // progress.
  if(EvaluateGoalConstraints(_p))
    return Complete;
  else
    return InProgress;
}

/*--------------------------- Evaluation Helpers -----------------------------*/

bool
MPTask::
EvaluateStartConstraints(const std::vector<Cfg>& _p) const {
  const bool ok = EvaluateConstraints(_p, m_startConstraints);
  if(ok and m_status == OnDeck)
    m_status = InProgress;
  return ok;
}


bool
MPTask::
EvaluatePathConstraints(const std::vector<Cfg>& _p) const {
  /// @TODO Consider some kind of caching mechanism to avoid extraneous
  ///       recomputation.
  return EvaluateConstraints(_p, m_pathConstraints);
}


bool
MPTask::
EvaluateGoalConstraints(const std::vector<Cfg>& _p) const {
  return EvaluateConstraints(_p, m_goalConstraints);
}

/*-------------------------------- Helpers -----------------------------------*/

bool
MPTask::
EvaluateConstraints(const std::vector<Cfg>& _p,
    const MPTask::ConstraintSet& _constraints) const {
  const auto& cfg = _p.back();
  for(const auto& constraint : _constraints)
    if(!constraint->Satisfied(cfg))
      return false;
  return true;
}


const Boundary*
MPTask::
MakeComposeBoundary(const MPTask::ConstraintSet& _constraints) const noexcept {
  /// @TODO Create a composed boundary from the constraint boundaries. We should
  ///       cache this to avoid re-building the boundary repeatedly.

  if(_constraints.empty())
    return nullptr;

  std::cout << "Warning: MPTask is currently creating constraint boundaries "
            << "from only the first constraint in each set!"
            << std::endl;
  return _constraints.front()->GetBoundary();
}

/*----------------------------------------------------------------------------*/
