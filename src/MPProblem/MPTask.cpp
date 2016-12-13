#include "MPTask.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/Constraints/ConstraintFactory.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"


/*------------------------------ Construction --------------------------------*/

MPTask::
MPTask(MPProblem* const _problem, XMLNode& _node) {
  // Parse task and robot labels.
  m_label = _node.Read("label", true, "", "Unique label for this task");
  m_robotLabel = _node.Read("robot", true, "", "Label for the robot assigned to"
      " this task.");

  // Get the robot by label.
  m_robot = _problem->GetNewRobot(m_robotLabel);
  auto mb = m_robot->GetMultiBody();

  // Parse constraints.
  ConstraintFactory factory;

  for(auto& typeNode : _node) {
    if(typeNode.Name() == "StartConstraints")
      for(auto& constraintNode : typeNode)
        m_startConstraints.push_back(factory(mb, constraintNode));
    else if(typeNode.Name() == "PathConstraints")
      for(auto& constraintNode : typeNode)
        m_pathConstraints.push_back(factory(mb, constraintNode));
    else if(typeNode.Name() == "GoalConstraints")
      for(auto& constraintNode : typeNode)
        m_goalConstraints.push_back(factory(mb, constraintNode));
  }
}


MPTask::
~MPTask() {
  for(auto c : m_startConstraints)
    delete c;
  for(auto c : m_pathConstraints)
    delete c;
  for(auto c : m_goalConstraints)
    delete c;
}

/*-------------------------- Constraint Accessors ----------------------------*/

void
MPTask::
AddStartConstraint(Constraint* const _c) {
  m_startConstraints.push_back(_c);
}


void
MPTask::
AddPathConstraint(Constraint* const _c) {
  m_pathConstraints.push_back(_c);
}


void
MPTask::
AddGoalConstraint(Constraint* const _c) {
  m_goalConstraints.push_back(_c);
}


const std::vector<Constraint*>&
MPTask::
GetStartConstraints() const noexcept {
  return m_startConstraints;
}


const std::vector<Constraint*>&
MPTask::
GetPathConstraints() const noexcept {
  return m_pathConstraints;
}


const std::vector<Constraint*>&
MPTask::
GetGoalConstraints() const noexcept {
  return m_goalConstraints;
}

/*---------------------------- Constraint Evaluation -------------------------*/

MPTask::Status
MPTask::
Evaluate(const std::vector<Cfg>& _p) const {
  // If start constraints are not satisfied, this task is still on deck.
  if(!EvaluateStartConstraints(_p))
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
  const auto& cfg = _p.front();
  for(const auto& constraint : m_startConstraints)
    if(!constraint->Satisfied(cfg))
      return false;
  return true;
}


bool
MPTask::
EvaluatePathConstraints(const std::vector<Cfg>& _p) const {
  /// @TODO Consider some kind of caching mechanism to avoid extraneous
  ///       recomputation.
  for(const auto& cfg : _p)
    for(const auto& constraint : m_pathConstraints)
      if(!constraint->Satisfied(cfg))
        return false;
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

/*----------------------------------------------------------------------------*/
