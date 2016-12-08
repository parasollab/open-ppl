#ifndef MP_TASK_TYPE_H_
#define MP_TASK_TYPE_H_

#include <string>
#include <vector>

#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Robot.h"
#include "MPTask/Constraints/ConstraintFactory.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"

class Robot;


////////////////////////////////////////////////////////////////////////////////
/// Describes a motion task with start/end conditions and constraints on
/// allowable trajectories.
///
/// @details Tasks are defined by a robot group and three sets of constraints:
///   @arg Start constraints describe the conditions required to start a task.
///   @arg Path constraints describe restrictions that must be observed
///        throughout a valid solution.
///   @arg Goal constraints describe the conditions that must be met to complete
///        a task.
/// The robot group is the group assigned to perform the task.
///
/// XML Parsing:
/// Each task has two required attributes and no optional attributes:
///   @arg label A unique label for the task.
///   @arg robot The label for the robot assigned to the task.
/// Each task also has three child nodes for the start, path, and goal
/// constraints, respectively. Each contains a set of its own child nodes
/// describing the appropriate constraints.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPTaskType final {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::PathType Path;
    typedef ConstraintType<MPTraits>    Constraint;

    ///@}
    ///@name Local Types
    ///@{

    /// The set of states that describe how well a solution path fullfills a
    /// task.
    /// @arg OnDeck The beginning of the path doesn't satisfy the start
    ///      constraints.
    /// @arg InProgress The beginning satisfies the start constraints. No path
    ///      constraints are violated, but the goal constraints are not
    ///      satisfied.
    /// @arg Complete All constraints are satisfied.
    /// @arg Invalid A path constraint is violated.
    enum Status {OnDeck, InProgress, Complete, Invalid};

    ///@}
    ///@name Construction
    ///@{

    /// Parse the set of task constraints described in an XML node.
    /// @param[in] _problem The MPProblem for this task.
    /// @param[in] _node The XML node to parse.
    MPTaskType(MPProblem* const _problem, XMLNode& _node);

    ~MPTaskType();

    ///@}
    ///@name Constraint Accessors
    ///@{
    /// The task will take ownership of any added constraints and delete them
    /// when necessary. We require dynamically-allocated objects here because
    /// exact (derived) type of each constraint will not be known until runtime.

    void AddStartConstraint(Constraint* const _c);
    void AddPathConstraint(Constraint* const _c);
    void AddGoalConstraint(Constraint* const _c);

    const std::vector<Constraint*>& GetStartConstraints() const noexcept;
    const std::vector<Constraint*>& GetPathConstraints() const noexcept;
    const std::vector<Constraint*>& GetGoalConstraints() const noexcept;

    ///@}
    ///@name Constraint Evaluation
    ///@{

    /// Evaluate a path to see if it meets the constraints.
    /// @param[in] _p The path to validate.
    /// @return The status of the task using _p as a solution.
    Status Evaluate(const Path& _p);

    /// Check if a path's starting point satisfies the start constraints.
    /// @param[in] _p The potential solution to check.
    /// @return True if the starting point of _p satisfies all start constraints.
    bool EvaluateStartConstraints(const Path& _p);

    /// Check if all points in the path satisfy the end constraints.
    /// @param[in] _p The potential solution to check.
    /// @return True if all points in _p satisfy all path constraints.
    bool EvaluatePathConstraints(const Path& _p);

    /// Check if a path's end point satisfies the goal constraints.
    /// @param[in] _p The potential solution to check.
    /// @return True if the ending point of _p satisfies all goal constraints.
    bool EvaluateGoalConstraints(const Path& _p);

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_label;          ///< The unique task label.
    std::string m_robotLabel;     ///< The robot (group) label.

    Robot* m_robot{nullptr};      ///< The robot (group) assigned to this task.
    /// @TODO Change this to a robot group when that code is ready.
    //RobotGroup* m_group{nullptr}; ///< The robot group assigned to this task.

    std::vector<Constraint*> m_startConstraints; ///< Req'd to start task.
    std::vector<Constraint*> m_pathConstraints;  ///< Req'd during whole task.
    std::vector<Constraint*> m_goalConstraints;  ///< Req'd to end task.

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MPTaskType<MPTraits>::
MPTaskType(MPProblem* const _problem, XMLNode& _node) {
  // Parse task and robot labels.
  m_label = _node.Read("label", true, "", "Unique label for this task");
  m_robotLabel = _node.Read("robot", true, "", "Label for the robot assigned to"
      " this task.");

  // Get the robot by label.
  m_robot = _problem->GetNewRobot(m_robotLabel);
  auto mb = m_robot->GetMultiBody();

  // Parse constraints.
  ConstraintFactory<MPTraits> factory;

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


template <typename MPTraits>
MPTaskType<MPTraits>::
~MPTaskType() {
  for(auto c : m_startConstraints)
    delete c;
  for(auto c : m_pathConstraints)
    delete c;
  for(auto c : m_goalConstraints)
    delete c;
}

/*-------------------------- Constraint Accessors ----------------------------*/

template <typename MPTraits>
void
MPTaskType<MPTraits>::
AddStartConstraint(Constraint* const _c) {
  m_startConstraints.push_back(_c);
}


template <typename MPTraits>
void
MPTaskType<MPTraits>::
AddPathConstraint(Constraint* const _c) {
  m_pathConstraints.push_back(_c);
}


template <typename MPTraits>
void
MPTaskType<MPTraits>::
AddGoalConstraint(Constraint* const _c) {
  m_goalConstraints.push_back(_c);
}


template <typename MPTraits>
const std::vector<typename MPTaskType<MPTraits>::Constraint*>&
MPTaskType<MPTraits>::
GetStartConstraints() const noexcept {
  return m_startConstraints;
}


template <typename MPTraits>
const std::vector<typename MPTaskType<MPTraits>::Constraint*>&
MPTaskType<MPTraits>::
GetPathConstraints() const noexcept {
  return m_pathConstraints;
}


template <typename MPTraits>
const std::vector<typename MPTaskType<MPTraits>::Constraint*>&
MPTaskType<MPTraits>::
GetGoalConstraints() const noexcept {
  return m_goalConstraints;
}

/*---------------------------- Constraint Evaluation -------------------------*/

template <typename MPTraits>
typename MPTaskType<MPTraits>::Status
MPTaskType<MPTraits>::
Evaluate(const Path& _p) {
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

template <typename MPTraits>
bool
MPTaskType<MPTraits>::
EvaluateStartConstraints(const Path& _p) {
  const auto& cfg = _p.front();
  for(const auto& constraint : m_startConstraints)
    if(!constraint(cfg))
      return false;
  return true;
}


template <typename MPTraits>
bool
MPTaskType<MPTraits>::
EvaluatePathConstraints(const Path& _p) {
  /// @TODO Consider some kind of caching mechanism to avoid extraneous
  ///       recomputation.
  for(const auto& cfg : _p)
    for(const auto& constraint : m_pathConstraints)
      if(!constraint(cfg))
        return false;
  return true;
}


template <typename MPTraits>
bool
MPTaskType<MPTraits>::
EvaluateGoalConstraints(const Path& _p) {
  const auto& cfg = _p.back();
  for(const auto& constraint : m_goalConstraints)
    if(!constraint(cfg))
      return false;
  return true;
}

/*----------------------------------------------------------------------------*/

#endif
