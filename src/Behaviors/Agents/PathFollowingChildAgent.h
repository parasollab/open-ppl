#ifndef PATH_FOLLOWING_CHILD_AGENT_H_
#define PATH_FOLLOWING_CHILD_AGENT_H_

#include "Agent.h"


#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"
#include "Battery.h"
#include "packet.h"

class AgentGroup;
////////////////////////////////////////////////////////////////////////////////
/// This agent calls pmpl once and then follows the resulting path.
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

    void InitializePointsVector();

    void SetMPRoadmap(RoadmapType* _solution);

    Cfg GetRandomRoadmapPoint();

    MPTask* GetNewTask();

    bool CallForHelp();

    Robot* GetNearestHelper();

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

    ///Get the information about coordinates and orientation of robot from
    ///markers
    vector<double> GetCoordinatesFromMarker();

    vector<double> GetRotationAndTranslationAmt(const Cfg&, const Cfg&);

    vector<double> GetOdometry();

    void SetOdometry(const vector<double>&);

    void UpdateOdometry(const double&, const double&, const double&);

    void CreateNewTask(Cfg& _start, Cfg& _goal, std::string _label);
   
    void SetPriority(size_t _priority);


    typedef RoadmapGraph<CfgType, WeightType>         GraphType;
    typedef typename GraphType::vertex_descriptor     VID;
    typedef typename std::vector<VID>::const_iterator VIDIterator;

    ///@}

  private :

    ///@name Helper Functions
    ///@{

    void WorkerStep(const double _dt);

    void HelperStep(const double _dt);

    bool AvoidCollision();

    double GetPathLength(vector<Cfg>& path);

    double EuclideanDistance(Cfg point1, Cfg point2);

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
    
    size_t m_avoidCollisionHalt{0}; //Used to have higher priority robot halt a single time in instance of collision
    //Think of better solution
    ///@}

};

#endif
