#ifndef MP_TASK_TYPE_H_
#define MP_TASK_TYPE_H_

#include <memory>
#include <string>
#include <vector>

class Boundary;
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

    /// A set of constraints.
    typedef std::vector<std::unique_ptr<Constraint>> ConstraintSet;

    ///@}
    ///@name Construction
    ///@{

    /// Create an empty task for a given robot.
    /// @param _robot The robot assigned to this task.
    explicit MPTask(Robot* const _robot);

    /// Parse the set of task constraints described in an XML node.
    /// @param _problem The MPProblem for this task.
    /// @param _node The XML node to parse.
    explicit MPTask(MPProblem* const _problem, XMLNode& _node);

    MPTask(const MPTask& _other);  ///< Copy.
    MPTask(MPTask&& _other);       ///< Move.

    ~MPTask();

    ///@}
    ///@name Assignment
    ///@{

    MPTask& operator=(const MPTask& _other); ///< Copy.
    MPTask& operator=(MPTask&& _other);      ///< Move.

    ///@}
    ///@name Property Accessors
    ///@{

    /// Get the robot associated with this task.
    Robot* GetRobot() const noexcept;

    /// Re-assign this task to another robot.
    /// @param _r The destination robot which will receive this assignment.
    void SetRobot(Robot* const _r);

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

    void SetStartConstraint(std::unique_ptr<Constraint>&& _c);
    void AddPathConstraint(std::unique_ptr<Constraint>&& _c);
    void AddGoalConstraint(std::unique_ptr<Constraint>&& _c);

    const Constraint* GetStartConstraint() const noexcept;
    const ConstraintSet& GetPathConstraints() const noexcept;
    const ConstraintSet& GetGoalConstraints() const noexcept;
    const double GetArrivalTime() const noexcept;
    const double GetStartTime() const noexcept;

    ///@}
    ///@name Time Accessors
    ///@{

    /// Stores the projected arrival time of the robot at the goal
    void SetArrivalTime(double _arrivalTime);

    /// Stores the expected start time of the task
    void SetStartTime(double _arrivalTime);

    ///@}
    ///@name Constraint Evaluation
    ///@{

    /// Evaluate a path to see if it meets the constraints.
    /// @param _p The path to validate.
    /// @return The status of the task using _p as a solution.
    Status Evaluate(const std::vector<Cfg>& _p) const;

    /// Check if a path's starting point satisfies the start constraints.
    /// @param _p The potential solution to check.
    /// @return True if the starting point of _p satisfies all start constraints.
    bool EvaluateStartConstraint(const std::vector<Cfg>& _p) const;

    /// Check if all points in the path satisfy the end constraints.
    /// @param _p The potential solution to check.
    /// @return True if all points in _p satisfy all path constraints.
    bool EvaluatePathConstraints(const std::vector<Cfg>& _p) const;

    /// Check if a path's end point satisfies the goal constraints.
    /// @param _p The potential solution to check.
    /// @return True if the ending point of _p satisfies all goal constraints.
    bool EvaluateGoalConstraints(const std::vector<Cfg>& _p) const;

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

    std::string GetCapability() const;

  private:

    ///@name Internal State
    ///@{

    /// @TODO Change this to a robot group when that code is ready.
    Robot* m_robot{nullptr};      ///< The robot assigned to this task.

    std::string m_label;          ///< The task's semantic label.

    std::string m_capability;  ///< Indicates the capability of the robot performing the task.

    mutable Status m_status{OnDeck};      ///< The status of the current task.

    std::unique_ptr<Constraint> m_startConstraint;  ///< Req'd to start task.
    ConstraintSet m_pathConstraints;   ///< Req'd during whole task.
    ConstraintSet m_goalConstraints;   ///< Req'd to end task.

    double m_arrivalTime{0}; ///< Estimated time of arrival at goal
    double m_startTime{0};   ///< Estimated start time of task
    ///@}

};

#endif
