#ifndef GROUP_TASK_H
#define GROUP_TASK_H

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "MPProblem/MPTask.h"

class Boundary;
class GroupCfg;
class Constraint;
class MPProblem;
class Robot;
class RobotGroup;
class XMLNode;


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
class GroupTask final {

  public:

    ///@name Local Types
    ///@{

    /// The set of states that describe how well a solution path fulfills a
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

    /// Create an empty task for a given robot.
    /// @param _robot The robot assigned to this task.
    explicit GroupTask(RobotGroup* const _robotGroup);

    /// Parse the set of task constraints described in an XML node.
    /// @param _problem The MPProblem for this task.
    /// @param _node The XML node to parse.
    explicit GroupTask(MPProblem* const _problem, XMLNode& _node);

    GroupTask(const GroupTask& _other);  ///< Copy.
    GroupTask(GroupTask&& _other);       ///< Move.

    ~GroupTask();

    ///@}
    ///@name Assignment
    ///@{

    GroupTask& operator=(const GroupTask& _other); ///< Copy.
    GroupTask& operator=(GroupTask&& _other);      ///< Move.

    ///@}
    ///@name Property Accessors
    ///@{

    /// Get the robot associated with this task.
    RobotGroup* GetRobotGroup() const noexcept;

    /// Assign this task to the robot group specified.
    /// @param _r The destination robot which will receive this assignment.
    void SetRobotGroup(RobotGroup* const _r);

    /// Get the semantic label for this task.
    const std::string& GetLabel() const noexcept;

    /// Set the semantic label for this task.
    void SetLabel(const std::string& _label) noexcept;

    ///@}
    ///@name Constraint Accessors
    ///@{
    /// The task will take ownership of any added constraints and delete them
    /// when necessary. We require dynamically-allocated objects here because
    /// exact (derived) type of each constraint will not be known until runtime.

    /// Uses the robot group in _center to populate all of the individual cfgs
    /// in that group cfg.
    void GetStartConstraintCenter(GroupCfg& _center) const noexcept;

//    void SetStartConstraint(std::unique_ptr<Constraint>&& _c);
//    void AddPathConstraint(std::unique_ptr<Constraint>&& _c);
//    void AddGoalConstraint(std::unique_ptr<Constraint>&& _c);
//    void SetArrivalTime(double _arrivalTime);

//    const Constraint* GetStartConstraint() const noexcept;
//    const ConstraintSet& GetPathConstraints() const noexcept;
//    const ConstraintSet& GetGoalConstraints() const noexcept;
//    const double GetArrivalTime() const noexcept;

    ///@}
    ///@name Constraint Evaluation
    ///@{

    /// Evaluate a path to see if it meets the constraints.
    /// @param _p The path to validate.
    /// @return The status of the task using _p as a solution.
    Status Evaluate(const std::vector<GroupCfg>& _p) const;

//    /// Check if a path's starting point satisfies the start constraints.
//    /// @param _p The potential solution to check.
//    /// @return True if the starting point of _p satisfies all start constraints.
//    bool EvaluateStartConstraint(const std::vector<Cfg>& _p) const;
//
//    /// Check if all points in the path satisfy the end constraints.
//    /// @param _p The potential solution to check.
//    /// @return True if all points in _p satisfy all path constraints.
//    bool EvaluatePathConstraints(const std::vector<Cfg>& _p) const;
//
//    /// Check if a path's end point satisfies the goal constraints.
//    /// @param _p The potential solution to check.
//    /// @return True if the ending point of _p satisfies all goal constraints.
//    bool EvaluateGoalConstraints(const std::vector<Cfg>& _p) const;

    ///@}
    ///@name Task Status
    ///@{
    /// Check/set the status of this task manually (as opposed to the Evaluate
    /// function).

    bool IsCompleted() const;
    void SetCompleted();

    bool IsStarted() const;
    void SetStarted();

    /// Reset the task as not started.
    void Reset();

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_label;          ///< The task's semantic label.

    RobotGroup* m_robotGroup{nullptr}; ///< The robot group assigned to this task.

    mutable Status m_status{OnDeck};      ///< The status of the group task.

//    double m_arrivalTime{0};

    /// TODO: Doesn't this need to be a map between a list of MPTasks??
    std::unordered_map<Robot*, MPTask> m_robotTasks; ///< The task for each robot in the group.

    ///@}

};

#endif
