#include "GroupTask.h"

#include "Geometry/Boundaries/Boundary.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include "Vector.h"


/*------------------------------ Construction --------------------------------*/

GroupTask::
GroupTask(RobotGroup* const _robot) : m_robotGroup(_robot) {
  m_label = "null group task";
}


GroupTask::
GroupTask(MPProblem* const _problem, XMLNode& _node) {
  // Parse task and robot labels.
  m_label = _node.Read("label", true, "", "Unique label for this task");

  // Get the robot group by label.
  const std::string robotGroupLabel = _node.Read("robotGroupLabel", true, "",
                            "Label for the robot group assigned to this task.");
  m_robotGroup = _problem->GetRobotGroup(robotGroupLabel);

  const std::string effectorGroupLabel = _node.Read("endEffectorGroupLabel",
           false, "",
           "Label for the robot group with manipulator assigned to this task.");
  if(!effectorGroupLabel.empty()) // Check if provided.
    m_endEffectorGroup = _problem->GetRobotGroup(effectorGroupLabel);

  const std::string manipGroupLabel = _node.Read("manipulatorGroupLabel", false,
      "", "Label for the robot group with manipulator assigned to this task.");
  if(!manipGroupLabel.empty()) // Check if provided.
    m_manipulatorGroup = _problem->GetRobotGroup(manipGroupLabel);

  // Parse constraints.
  for(auto& child : _node) {
    if(child.Name() == "Task") {
      MPTask robotTask(_problem, child); /// Build the task
      Robot* const robot = robotTask.GetRobot();
      if(!m_robotGroup->VerifyRobotInGroup(robot))
        throw ParseException(WHERE, "Robot with label " + robot->GetLabel() +
                                    " not in robot group with label " +
                                    m_robotGroup->GetLabel());
      m_robotTasks.emplace(robot, robotTask); /// Keyed on robot pointer
    }
    else
      throw ParseException(WHERE, "Invalid child node in Group Task.");
  }
}


GroupTask::
GroupTask(const GroupTask& _other) {
  *this = _other;
}


GroupTask::
GroupTask(GroupTask&& _other) = default;


GroupTask::
~GroupTask() = default;

/*-------------------------------- Assignment --------------------------------*/

GroupTask&
GroupTask::
operator=(const GroupTask& _other) {
  if(this != &_other) {
    m_label = _other.m_label;
    m_robotGroup = _other.m_robotGroup;
    m_robotTasks = _other.m_robotTasks;
  }
  return *this;
}


GroupTask&
GroupTask::
operator=(GroupTask&& _other) = default;

/*--------------------------- Property Accessors -----------------------------*/

RobotGroup*
GroupTask::
GetRobotGroup() const noexcept {
  return m_robotGroup;
}


RobotGroup*
GroupTask::
GetEndEffectorGroup() const noexcept {
  return m_endEffectorGroup;
}


RobotGroup*
GroupTask::
GetManipulatorGroup() const noexcept {
  return m_manipulatorGroup;
}


void
GroupTask::
SetRobotGroup(RobotGroup* const _r) {
  if(m_robotGroup == _r)
    return;
  else if(!m_robotGroup)
    throw RunTimeException(WHERE, "Re-assigning of group tasks currently is not"
                                  " functional!");

  m_robotGroup = _r;
}


const std::string&
GroupTask::
GetLabel() const noexcept {
  return m_label;
}


void
GroupTask::
SetLabel(const std::string& _label) noexcept {
  m_label = _label;
}


Robot*
GroupTask::
GetManipulatorRobot() {
  if(!m_manipulatorGroup)
    return nullptr;

  // If the manipulator group is present, return the manipulator.
  return m_manipulatorGroup->GetRobot("manipulator");
}


Robot*
GroupTask::
GetEndEffectorRobot() {
  if(!m_manipulatorGroup)
    return nullptr;

  // If the manipulator group is present, return the manipulator.
  return m_endEffectorGroup->GetRobot("effector");
}


/*-------------------------- Constraint Accessors ----------------------------*/

void
GroupTask::
GetStartConstraintCenter(GroupCfg& _center) const noexcept {
  for(Robot* const robot : _center.GetRobots()) {
    Cfg robotCfg(robot);
    robotCfg.SetData(m_robotTasks.at(robot).GetStartConstraint()->
                                                    GetBoundary()->GetCenter());
    _center.SetRobotCfg(robot, std::move(robotCfg));
  }
}


//void
//GroupTask::
//SetStartConstraint(std::unique_ptr<Constraint>&& _c) {
//  m_startConstraint = std::move(_c);
//}
//
//
//void
//GroupTask::
//AddPathConstraint(std::unique_ptr<Constraint>&& _c) {
//  m_pathConstraints.push_back(std::move(_c));
//}
//
//
//void
//GroupTask::
//AddGoalConstraint(std::unique_ptr<Constraint>&& _c) {
//  m_goalConstraints.push_back(std::move(_c));
//}
//
//
//void
//GroupTask::
//SetArrivalTime(double _arrivalTime){
//  m_arrivalTime = _arrivalTime;
//}
//
//
//const Constraint*
//GroupTask::
//GetStartConstraint() const noexcept {
//  return m_startConstraint.get();
//}
//
//
//const GroupTask::ConstraintSet&
//GroupTask::
//GetPathConstraints() const noexcept {
//  return m_pathConstraints;
//}
//
//
//const GroupTask::ConstraintSet&
//GroupTask::
//GetGoalConstraints() const noexcept {
//  return m_goalConstraints;
//}
//
//
//const double
//GroupTask::
//GetArrivalTime() const noexcept {
//  return m_arrivalTime;
//}


/*------------------------------- Task Status --------------------------------*/

bool
GroupTask::
IsCompleted() const {
  return m_status == Complete;
}


void
GroupTask::
SetCompleted() {
  m_status = Complete;
}


bool
GroupTask::
IsStarted() const {
  return m_status == InProgress or IsCompleted();
}


void
GroupTask::
SetStarted() {
  if(!IsStarted())
    m_status = InProgress;
}


void
GroupTask::
Reset() {
  for(auto& taskPair : m_robotTasks)
    taskPair.second.Reset(); /// TODO: loop through LIST of MPTasks to reset!
}

/*---------------------------- Constraint Evaluation -------------------------*/

GroupTask::Status
GroupTask::
Evaluate(const std::vector<GroupCfg>& _p) const {

  /// TODO: Loop through all MPTasks after deciding what should be done here!
  throw RunTimeException(WHERE, "Need to decide what evaluating a group task entails!");

//  // If start constraints are not satisfied, this task is still on deck.
//  if(m_status == OnDeck and !EvaluateStartConstraint(_p))
//    return OnDeck;
//
//  // If path constraints aren't satisfied, this is an invalid path.
//  if(!EvaluatePathConstraints(_p))
//    return Invalid;
//
//  // If goal constraints are satisfied, task as finished. Otherwise, it's in
//  // progress.
//  if(EvaluateGoalConstraints(_p))
//    return Complete;
//  else
//    return InProgress;
}

/*--------------------------- Evaluation Helpers -----------------------------*/

//bool
//GroupTask::
//EvaluateStartConstraint(const std::vector<Cfg>& _p) const {
//  const bool ok = !m_startConstraint.get() or m_startConstraint->Satisfied(_p.front());
//  if(ok and m_status == OnDeck)
//    m_status = InProgress;
//  return ok;
//}
//
//
//bool
//GroupTask::
//EvaluatePathConstraints(const std::vector<Cfg>& _p) const {
//  /// @TODO Consider some kind of caching mechanism to avoid extraneous
//  ///       recomputation.
//  for(const auto& constraint : m_pathConstraints){
//    for(const auto& cfg : _p){
//      if(!constraint->Satisfied(cfg))
//        return false;
//    }
//  }
//  return true;
//}
//
//
//bool
//GroupTask::
//EvaluateGoalConstraints(const std::vector<Cfg>& _p) const {
//  const auto& cfg = _p.back();
//  for(const auto& constraint : m_goalConstraints)
//    if(!constraint->Satisfied(cfg))
//      return false;
//  return true;
//}
