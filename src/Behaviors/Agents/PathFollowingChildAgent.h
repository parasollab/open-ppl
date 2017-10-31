#ifndef PATH_FOLLOWING_CHILD_AGENT_H_
#define PATH_FOLLOWING_CHILD_AGENT_H_

#include "Agent.h"


#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"
#include "Battery.h"
#include "packet.h"
#include "MPProblem/Robot/HardwareInterfaces/HardwareInterface.h"
#include "MPProblem/Robot/HardwareInterfaces/NetbookInterface.h"

class AgentGroup;
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

    /// Call PMPL to create a path for the agent to follow.
    virtual void Initialize() override;

    /// Follow the path.
    virtual void Step(const double _dt) override;

    /// Clean up.
    virtual void Uninitialize() override;

    ///Initialize the mp solution object
    void InitializeMpSolution(MPSolution*);

    //TEMP: Initialize goals to visit. TODO:
    //Add this to the group agent class.
    void InitializePointsVector();

    void SetMPRoadmap(RoadmapType* _solution);

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

    AgentGroup* m_parentAgent{nullptr};

    NetbookInterface* m_netbook;

    typedef RoadmapGraph<CfgType, WeightType>         GraphType;
    typedef typename GraphType::vertex_descriptor     VID;
    typedef typename std::vector<VID>::const_iterator VIDIterator;

  

    ///@}

  private :

    ///@name Helper Functions
    ///@{

    void WorkerStep();

    void HelperStep();

    bool AvoidCollision();

    double GetPathLength(const vector<Cfg>& path);

    double EuclideanDistance(const Cfg& point1, const Cfg& point2);

    //@}

  protected:

    ///@name Internal State
    ///@{

    std::vector<Cfg> m_path; ///< The path to follow.
    size_t m_pathIndex{0};   ///< The path node that is the current subgoal.

    MPLibrary* m_library{nullptr}; ///< This agent's planning library.

    MPSolution* m_solution{nullptr}; ///< The solution with the roadmap to follow.

    //MPTask* m_task{nullptr};

    static vector<Cfg> m_AllRoadmapPoints;

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
