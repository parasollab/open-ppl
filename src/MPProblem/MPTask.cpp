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
        m_startConstraint = Constraint::Factory(m_robot, grandChild);
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
    if(_other.m_startConstraint.get())
      m_startConstraint = _other.m_startConstraint->Clone();
    else 
      m_startConstraint.reset();
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

  if(m_startConstraint.get())
    m_startConstraint->SetRobot(_r);
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
SetStartConstraint(std::unique_ptr<Constraint>&& _c) {
  m_startConstraint = std::move(_c);
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

const Constraint*
MPTask::
GetStartConstraint() const noexcept {
  return m_startConstraint.get();
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
  if(m_status == OnDeck and !EvaluateStartConstraint(_p))
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
EvaluateStartConstraint(const std::vector<Cfg>& _p) const {
  const bool ok = !m_startConstraint.get() or m_startConstraint->Satisfied(_p.front());
  if(ok and m_status == OnDeck)
    m_status = InProgress;
  return ok;
}


bool
MPTask::
EvaluatePathConstraints(const std::vector<Cfg>& _p) const {
  /// @TODO Consider some kind of caching mechanism to avoid extraneous
  ///       recomputation.
  for(const auto& constraint : m_pathConstraints){
    for(const auto& cfg : _p){
      if(!constraint->Satisfied(cfg))
        return false;
    }
  }
  return true;
}


bool
MPTask::
EvaluateGoalConstraints(const std::vector<Cfg>& _p) const {
  const auto& cfg = _p.back();
  for(const auto& constraint : m_goalConstraints)
    if(!constraint->Satisfied(cfg))
      return false;
  return true;
}
