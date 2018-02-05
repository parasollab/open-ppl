#ifndef PATH_FOLLOWING_CHILD_AGENT_H_
#define PATH_FOLLOWING_CHILD_AGENT_H_

#include "Agent.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"

class BatteryConstrainedGroup;
class NetbookInterface;


////////////////////////////////////////////////////////////////////////////////
/// This agent follows a set of tasks and executes the helper worker behavior.
////////////////////////////////////////////////////////////////////////////////
class PathFollowingChildAgent : public Agent {

  public:

    ///@name Construction
    ///@{

    PathFollowingChildAgent(Robot* const _r);

    PathFollowingChildAgent(Robot* const _r, XMLNode& _node);

    virtual ~PathFollowingChildAgent();

    ///@}
    ///@name Agent Interface
    ///@{

    virtual void Initialize() override;

    virtual void Step(const double _dt) override;

    virtual void Uninitialize() override;

    ///@}
    ///@name Child Interface
    ///@{

    /// Check if the agent's battery is considered to be below the 'low'
    /// threshold.
    bool IsBatteryLow();

    /// Check if the agent's battery is considered to be above the 'high'
    /// threshold.
    bool IsBatteryLow();


    /// Get the next path for this child. Sets m_path.
    /// @param _task The task defining the next goal.
    /// @param _collisionAvoidance Is this a collision avoidance path?
    void GetNextPath(MPTask* const _task, const bool _collisionAvoidance = false);

    const bool CallForHelp();

    void ExecuteTask(double _dt);

    const bool IsHeadOnCollision();

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    /// Check if this member is in collision proximity to any others. If so, ask
    /// the coordinator to arbitrate the collision.
    void InCollision();

    /// Generate a new plan for the robot's current task.
    void GeneratePlan();

    void AvoidCollision();

    void Rotate(double&, double);

    /// Sum the length across an entire path.
    /// @param _path The path.
    /// @return The length of _path from start to goal.
    const double GetPathLength(const vector<Cfg>& _path) const;

    void PauseAgent(double);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    BatteryConstrainedGroup* m_parentAgent{nullptr};

    MPLibrary* m_library{nullptr}; ///< This agent's planning library.

    std::vector<Cfg> m_path; ///< The path to follow.
    size_t m_pathIndex{0};   ///< The path node that is the current subgoal.




    bool m_shouldHalt{false}; ///< The robot should halt if inCollision & lower priority.

    double m_distance{0.0}; ///< The distance traveled since localizing.


    bool m_ignoreLocalization{false};  ///Using this to ignore positional localization when trying to localize the angle

    bool m_ignoreAngleLocalization{true};

    bool m_finishedLocalizing{true};

    Robot* m_headOnCollidingRobot{nullptr};

    Cfg m_robotPos; ///< Position of the physical robot

    bool m_goToSameGoal{true};   ///< Should the agent go to the same goal after replanning?

    ///@}

};

#endif
