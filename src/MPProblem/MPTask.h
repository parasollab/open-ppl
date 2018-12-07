#ifndef PMPL_MP_TASK_TYPE_H_
#define PMPL_MP_TASK_TYPE_H_

#include "nonstd/status.h"

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
/// Describes a motion task for a single robot in terms of start/goal conditions
/// and constraints on allowable trajectories.
///
/// @details Tasks are defined by a robot and three sets of constraints:
///   @arg Start constraints describe the conditions required to start a task.
///   @arg Path constraints describe restrictions that must be observed
///        throughout a valid solution.
///   @arg Goal constraints describe the conditions that must be met to complete
///        a task.
/// A specific robot may be assigned to perform the task, or null may be used to
/// indicate that any robot may perform it. A specific capability may also be
/// required.
///
/// @todo We should remove the capability here and redefine it as a constraint
///       (since the constraints should fully define the task). The fact that a
///       capability is or is not required should generally be discerned from
///       the problem, while a capability constraint would more likely represent
///       a deliberate restriction that is not strictly required (perhaps to
///       express an operational preference). We will also eventually encounter
///       cases where multiple capabilities are required to satsify a task.
///
/// @todo Support the use of path constraints in the library code, which
///       currently doesn't care about this.
////////////////////////////////////////////////////////////////////////////////
class MPTask final {

  public:

    ///@name Local Types
    ///@{

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

    /// Get the status object for this task.
    nonstd::status& GetStatus() noexcept;
    const nonstd::status& GetStatus() const noexcept;

    ///@}
    ///@name Constraint Accessors
    ///@{
    /// The task will take ownership of any added constraints and delete them
    /// when necessary.

    void SetStartConstraint(std::unique_ptr<Constraint>&& _c);
    void AddPathConstraint(std::unique_ptr<Constraint>&& _c);
    void AddGoalConstraint(std::unique_ptr<Constraint>&& _c);

    const Constraint* GetStartConstraint() const noexcept;
    const ConstraintSet& GetPathConstraints() const noexcept;
    const ConstraintSet& GetGoalConstraints() const noexcept;

    /// A task is considered to be 'empty' if it has neither start nor goal
    /// constraints.
    bool Empty() const noexcept;

    /// Get the number of goals in this task.
    size_t GetNumGoals() const noexcept;

    /// Sets the capability required for this task.
    /// @param _capability The capability required to complete this task.
    void SetCapability(const std::string& _capability);

    /// Get the robot capability which is required to satisfy this task, if any.
    /// @return The required capability for this task, or empty string if none.
    const std::string& GetCapability() const noexcept;

    ///@}
    ///@name Time Accessors
    ///@{
    /// Store and fetch estimated times for beginning and completing this task.

    void SetEstimatedStartTime(const double _time) noexcept;
    void SetEstimatedCompletionTime(const double _time) noexcept;

    double GetEstimatedStartTime() const noexcept;
    double GetEstimatedCompletionTime() const noexcept;

    ///@}
    ///@name Constraint Evaluation
    ///@{

    /// Evaluate whether a robot has the needed capability.
    /// @param _r The robot to check.
    /// @return True if _r meets any capability requirements.
    bool EvaluateCapability(const Robot* const _r) const;

    /// Check if a configuration satisfies the start constraints.
    /// @param _cfg The configuration to check.
    /// @return True if _cfg satisfies all start constraints.
    bool EvaluateStartConstraints(const Cfg& _cfg) const;

    /// Check if a configuration satisfies the start constraints.
    /// @param _cfg The configuration to check.
    /// @return True if _cfg satisfies all path constraints.
    bool EvaluatePathConstraints(const Cfg& _cfg) const;

    /// Check if a configuration satisfies the last goal constraints.
    /// @param _cfg The configuration to check.
    /// @return True if _cfg satisfies the last goal constraints.
    bool EvaluateGoalConstraints(const Cfg& _cfg) const;

    /// Check if a configuration satisfies the constraints for a specific goal.
    /// @param _cfg The configuration to check.
    /// @param _index The goal index to check.
    /// @return True if _cfg satisfies the last goal constraints.
    bool EvaluateGoalConstraints(const Cfg& _cfg, const size_t _index) const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    Robot* m_robot{nullptr};     ///< The robot assigned to this task.

    std::string m_label;         ///< The task's semantic label.
    nonstd::status m_status;     ///< The completeion status of the task.

    std::unique_ptr<Constraint> m_startConstraint; ///< Req'd to start task.
    ConstraintSet m_pathConstraints;               ///< Req'd during whole task.
    ConstraintSet m_goalConstraints;               ///< Req'd to end task.
    std::string m_capability;    ///< A capability required to perform the task.

    double m_startTime{0};   ///< Estimated start time of task
    double m_finishTime{0};  ///< Estimated time of arrival at goal

    ///@}

};

#endif
