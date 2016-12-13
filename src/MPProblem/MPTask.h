#ifndef MP_TASK_TYPE_H_
#define MP_TASK_TYPE_H_

#include <string>
#include <vector>

class Cfg;
class Constraint;
class MPProblem;
class Robot;
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
class MPTask final {

  public:

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
    MPTask(MPProblem* const _problem, XMLNode& _node);

    ~MPTask();

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
    Status Evaluate(const std::vector<Cfg>& _p) const;

    /// Check if a path's starting point satisfies the start constraints.
    /// @param[in] _p The potential solution to check.
    /// @return True if the starting point of _p satisfies all start constraints.
    bool EvaluateStartConstraints(const std::vector<Cfg>& _p) const;

    /// Check if all points in the path satisfy the end constraints.
    /// @param[in] _p The potential solution to check.
    /// @return True if all points in _p satisfy all path constraints.
    bool EvaluatePathConstraints(const std::vector<Cfg>& _p) const;

    /// Check if a path's end point satisfies the goal constraints.
    /// @param[in] _p The potential solution to check.
    /// @return True if the ending point of _p satisfies all goal constraints.
    bool EvaluateGoalConstraints(const std::vector<Cfg>& _p) const;

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

#endif
