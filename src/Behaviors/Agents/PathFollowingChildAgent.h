#ifndef PATH_FOLLOWING_CHILD_AGENT_H_
#define PATH_FOLLOWING_CHILD_AGENT_H_

#include "Agent.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"
#include "Battery.h"
#include "packet.h"

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

    /// Initializing the solution for this robot based on the parent's
    /// shared roadmap.
    /// @param _s The parent's solution object.
    void InitializeMPSolution(MPSolution* const _s);

    Cfg GetRandomRoadmapPoint();

    MPTask* GetNewTask();

    bool CallForHelp();

    int GetNearestHelper();

    bool IsAtChargingStation();

    void ClearChargingStation();

    void FindNearestChargingLocation();

    void ExecuteTask(double _dt);

    bool InCollision();

    void SetParentRobot(Robot*);

    Robot* GetParentRobot();

    //TODO: move this to protected and add getters and setters.
    Robot* m_parentRobot{nullptr};

    BatteryConstrainedGroup* m_parentAgent{nullptr};

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    void WorkerStep();

    void HelperStep();

    bool AvoidCollision();

    /// Sum the length across an entire path.
    /// @param _path The path.
    /// @return The length of _path from start to goal.
    double GetPathLength(const vector<Cfg>& _path) const;

    /// Compute the euclidean distance in the XY plane between two
    /// configurations (ignoring Z and any rotations).
    /// @param _point1 The first cfg.
    /// @param _point1 The second cfg.
    /// @return The distance.
    /// @TODO This can be done with the existing WeightedEuclideanDistance
    ///       metric, should replace with that.
    double EuclideanDistance(const Cfg& _point1, const Cfg& _point2) const;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    std::vector<Cfg> m_path; ///< The path to follow.
    size_t m_pathIndex{0};   ///< The path node that is the current subgoal.

    MPLibrary* m_library{nullptr}; ///< This agent's planning library.

    MPSolution* m_solution{nullptr}; ///< The solution with the roadmap to follow.

    //MPTask* m_task{nullptr};

    static unordered_map<Robot*, Cfg> m_HelpersAvailable;

    bool m_done{false};

    std::vector<Cfg> m_goalTaken;

    Battery* m_battery{nullptr};

    std::vector<double> m_odometry{0.0,0.0,0.0};

    Robot* m_myHelper{nullptr};

    bool m_waitForHelp{true};

    bool m_shouldHalt{false}; ///< The robot should halt if inCollision & lower priority.

    bool m_waitingForHardware{false};  ///< Wait for the hardware to send back information (marker info)

    size_t m_avoidCollisionHalt{0}; //Used to have higher priority robot halt a single time in instance of collision
    //Think of better solution

    double m_dt{0.0};                         ///< Track the amount of _dt steps taken

    ///@}

};

#endif
