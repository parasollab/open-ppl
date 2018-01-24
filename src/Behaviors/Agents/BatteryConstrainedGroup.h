#ifndef BATTERY_CONSTRAINED_GROUP_H_
#define BATTERY_CONSTRAINED_GROUP_H_

#include "Agent.h"

#include "ConfigurationSpace/Cfg.h"
#include "Behaviors/Agents/PathFollowingChildAgent.h"
#include "MPLibrary/PMPL.h"


////////////////////////////////////////////////////////////////////////////////
/// This agent represents the task hand-off behavior presented in
/// the thesis Titled:
///
/// A Task Hand-Off Framework for Multi-Robot Systems by Saurabh Mishra
///
/// A portion of this work was also presented in:
/// Mishra, Saurabh, Samuel Rodriguez, Marco Morales, and Nancy M. Amato.
// "Battery-constrained coverage." In Automation Science and Engineering (CASE),
//  2016 IEEE International Conference on, pp. 695-700. IEEE, 2016.
///
/// This agent represents a coordinator and does not correspond to a physical
/// robot. Its job is to coordinate the children and hold shared data structures
/// for their use. It has a robot pointer purely for the purpose of building the
/// shared roadmap.
///
/// @WARNING This object currently only supports homogeneous robot teams, and
///          assumes a shared roadmap model.
////////////////////////////////////////////////////////////////////////////////
class BatteryConstrainedGroup : public Agent {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef RoadmapGraph<CfgType, WeightType>         GraphType;
    typedef typename GraphType::vertex_descriptor     VID;
    typedef typename std::vector<VID>::const_iterator VIDIterator;

    ///@}
    ///@name Construction
    ///@{

    /// Create an agent group with some robot as the coordinator.
    /// @param _r The coordinator robot.
    BatteryConstrainedGroup(Robot* const _r);

    virtual ~BatteryConstrainedGroup();

    ///@}
    ///@name Agent Interface
    ///@{

    /// Call PMPL to create a path for the agent to follow.
    virtual void Initialize() override;

    /// Follow the roadmap.
    virtual void Step(const double _dt) override;

    /// Clean up.
    virtual void Uninitialize() override;

    ///@}
    ///@name Coordinator Interface
    ///@{

    std::vector<Cfg> MakeNextPlan(MPTask* const _task, const bool _collisionAvoidance = false);

    /// Get a helper robot for the worker robot
    std::vector<Robot*>& GetHelpers();

    /// Get the charging locations for problem
    std::vector<pair<Cfg, bool>>& GetChargingLocations();

    /// Check a robot label to see if it is currently a helper.
    bool IsHelper(Robot* const _r) const;

    /// Get a random unvisited vertex from the roadmap.
    const Cfg GetRandomRoadmapPoint(std::string _label);

    /// Change the worker robot to a helper
    void SetHelper(Robot* _r);

    /// Change the worker robot to a helper
    void SetWorker(Robot* _r);

    /// Add a goal for a given robot
    void AddGoal(Cfg& _cfg, const std::string& _robotLabel);

    /// See if all the goals have been compeleted
    bool AllGoalsCompleted();
    ///@}

  private:

    ///@name Coordinator Helpers
    ///@{

    /// Set the next task for each child agent.
    void SetNextChildTask();

    /// Get all robots in the problem that are not the coordinator agent.
    std::vector<Robot*> GetChildRobots() const;

    /// Create a vector of all cfgs that are not yet visited. We want to visit
    /// each one exactly once with any one worker robot.
    void InitializeUnvisitedCfgs();

    ///@}

  protected:

    ///@name Internal State
    ///@{

    MPLibrary* m_library{nullptr};   ///< The shared-roadmap planning library.

    MPSolution* m_solution{nullptr}; ///< The shared-roadmap solution.

    std::vector<PathFollowingChildAgent*> m_childAgents;  ///< All robots in the group.

    std::vector<pair<Cfg, bool>> m_chargingLocations;  ///< pair of <chargingLocation, isFree>

    /// The helper robots which are currently idle.
    std::vector<Robot*> m_availableHelpers;

    /// The set of Cfgs that need to be visited by a worker.
    std::map<std::string, std::vector<Cfg> > m_unvisitedCfgs;


    ///Timers
    double m_lazyTime{0.0};

    double m_prmTime{0.0};
    ///@}

};

#endif
