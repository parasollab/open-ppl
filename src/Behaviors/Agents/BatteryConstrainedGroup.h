#ifndef BATTERY_CONSTRAINED_GROUP_H_
#define BATTERY_CONSTRAINED_GROUP_H_

#include "Agent.h"
#include "PlanningAgent.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"
#include "BatteryBreak.h"


////////////////////////////////////////////////////////////////////////////////
/// This agent represents the task hand-off behavior presented in
/// the thesis Titled:
///
/// A Task Hand-Off Framework for Multi-Robot Systems by Saurabh Mishra
///
/// A portion of this work was also presented in:
/// Mishra, Saurabh, Samuel Rodriguez, Marco Morales, and Nancy M. Amato.
/// "Battery-constrained coverage." In Automation Science and Engineering (CASE),
///  2016 IEEE International Conference on, pp. 695-700. IEEE, 2016.
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

    ///@name Local Types
    ///@{

    enum Role {Worker, Helper, Charging, ReturningToCharge, WaitingForHelp, GoingToHelp};

    struct PausedTask {

      std::shared_ptr<MPTask> m_task;
      Agent* m_previousOwner{nullptr};
      size_t m_priority{0};
    };

    ///@}
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

    BatteryConstrainedGroup(Robot* const _r, XMLNode& _node);

    virtual ~BatteryConstrainedGroup();

    virtual std::unique_ptr<Agent> Clone(Robot* const _r) const;
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

    /// Assign a new task to a group member.
    /// @param _member The member which is requesting a new task.
    void AssignTask(Agent* const _member);

    /// Save battery breaks after workers plan paths.
    void SetBatteryBreak(BatteryBreak _break, Agent* _member);

    /// Sends the current agent to its nearest charging location.
    /// @param _member The agent being sent to the charging location.
    void GoToCharge(Agent* const _member);
    
    /// Checks if the agent just swapped with another agent that is currently
    /// sitting on the goal of the task that _member just took over.
    /// @param _member The agent that is trying to plan.
    bool ClearToPlan(Agent* const _member);

    /// Decide what to do when some group of agents are facing a potential
    /// collision. Send a command to each agent in this group to resolve the
    /// potential collision.
    void ArbitrateCollision();

    ///@}
    ///@name Accessors
    ///@{

    /// Gets the current time of the simulation.
    /// @return m_currentTime
    double GetCurrentTime();

    ///@}

    
  private:

    ///@}
    ///@name Member Management
    ///@{

    /// Set the priority of a member agent.
    /// @param _a The member agent to set priority for.
    /// @param _priority The priority value.
    void SetPriority(Agent* const _a, const size_t _priority);

    /// Get the priority of a member agent.
    /// @param _a The member agent to set priority for.
    /// @return The priority of _a.
    size_t GetPriority(Agent* const _a);

    /// Determine if an agent has the highest priority in its proximity group.
    /// @param _a The member agent to check the priority for.
    /// @param _group The group of agents near _a.
    /// @return True if _a has a higher priority than the robots around it,
    /// False otherwise. 
    bool IsHighestPriority(Agent* const _a, const vector<Agent*>& _group);

    /// Change the role of a group member.
    /// @param _member The member agent.
    /// @param _role The new role for _member.
    void SetRole(Agent* const _member, const Role _role);

    /// Get the current role for a group member.
    /// @param _member The member agent.
    /// @return The current role of _member.
    Role GetRole(Agent* const _member) const;

    /// Determine if the group member is a worker.
    /// @param _member The member agent.
    /// @return True if the member is a worker.
    bool IsWorker(Agent* const _member) const;

    /// Get the set of all robots which are currently workers.
    std::vector<Agent*> GetWorkers();

    /// Get the set of all robots which are currently helpers.
    std::vector<Agent*> GetHelpers();

    /// Get the nearest helper to another group member.
    /// @param _member The target group member.
    /// @return The nearest helper to _member.
    Agent* GetNearestHelper(Agent* const _member);

    /// Send a group member to a designated location by assigning a new task.
    /// The group member still needs to plan for the new task with PMPL.
    /// @param _member The group member to dispatch.
    /// @param _where The location to send _member.
    void DispatchTo(Agent* const _member, std::unique_ptr<Boundary>&& _where);

    /// Check whether two group members are close enough to execute a task
    /// handoff.
    /// @param _member1 The first member.
    /// @param _member2 The second member.
    /// @return True if the members are close enough to execute a handoff.
    bool InHandoffProximity(Agent* const _member1, Agent* const _member2);

    /// Update other agents' knowledge about an agent's plan versions.
    /// @param _member The agent being updated.
    /// @param _agents The agents within proximity to the updating agent. 
    void UpdateVersionMap(Agent* const _member, std::vector<Agent*> _agents);

    /// For each agent in proximity, ensure that its map has an up-to-date
    /// version for the agent (_member) that may need to replan. 
    /// @param _member The agent who's version number must be verified for the
    /// proximity agents.
    /// @param _agents The proximity agents who's _member version number must be
    /// checked. 
    /// @return True if the versions are up-to-date, false otherwise.
    bool ValidateVersionMap(Agent* const _member, std::vector<Agent*> _agents);

    /// Checks if the agent(helper) has reached its assigned worker, and assigns
    /// it the new tasks if it has.
    /// @param _member The helper that is checking if it has reached the worker.
    void CheckReachedWorker(Agent* const _member);

    ///@}
    ///@name Charging Locations
    ///@{

    /// Create the set of charging locations.
    /// @TODO Currently we assume that the starting location for each helper is a
    ///       charging location. We need to implement a set of labeled regions
    ///       in a problem to define charging areas independently.
    /// @TODO We currently assume that charging locations are XY regions which
    ///       extend infinitely in the z-dimension.
    void InitializeChargingLocations();

    /// Locate the nearest unoccupied charging location to a given group member.
    /// @param _a The group member.
    /// @return A pointer to the charging region nearest to _a.
    std::pair<Boundary*, double> FindNearestChargingLocation(Agent* const _a);

    /// Determine whether a group member is at a charging location.
    /// @param _member The group member.
    /// @return A pointer to the nearest charging location within the threshold,
    ///         or null if none was found.
    bool IsAtChargingLocation(Agent* const _member);

    /// Clears the charging location that a member was previously on before
    /// going to assist a low-battery worker.
    /// @param _member The agent that was on the charging location
    void ClearChargingLocation(Agent* const _member);

    ///@}
    ///@name Task Management
    /// Assign a new task to a worker member.
    /// @param _member The worker member which is requesting a new task.
    void AssignTaskWorker(Agent* const _member);

    /// Assign a new task to a worker member.
    /// @param _member The worker member which is requesting a new task.
    void AssignTaskHelper(Agent* const _member);

    void AssignProactiveHelperTask(Agent* const _member);

    /// Assign a new task to a worker member.
    /// @param _member The worker member which is requesting a new task.
    void AssignTaskWaiting(Agent* const _member);


    void PerformHelperSwap(Agent* const _member, Agent* _worker);

    /// Checks the battery level of a worker and assigns its task to the
    /// PausedTask queue if the battery level is low.
    /// @param _member The worker member which could be pausing its task.
    void BatteryCheck(Agent* const _member);

    bool IsProactive();
    
    
    ///@}

  protected:

    ///@name Internal State
    ///@{

    MPLibrary* m_library{nullptr};   ///< The shared-roadmap planning library.

    MPSolution* m_solution{nullptr}; ///< The shared-roadmap solution.

    std::vector<std::string> m_memberLabels;  ///< Labels for the group members.
    std::vector<Agent*> m_memberAgents;       ///< All robots in the group.

    /// The relative priorities for member robots.
    std::unordered_map<Agent*, size_t> m_memberPriorities;

    /// The set of charging locations and the group member which occupies it, if
    /// any.
    std::vector<std::pair<std::unique_ptr<Boundary>, Agent*>> m_chargingLocations;

    /// Timers
    double m_lazyTime{0.0};
    double m_prmTime{0.0};

    /// Map group members to their current role.
    std::unordered_map<Agent*, Role> m_roleMap;

    /// Map group members to their last assigned task;
    std::list<PausedTask> m_pausedTasks;
    
    /// Map worker to assigned helper.
    std::unordered_map<Agent*, Agent*> m_helperMap;

    /// Map group members to their initial roles (needed for parsing only).
    std::unordered_map<std::string, Role> m_initialRoles;

    /// Flag determining whether or not the Agents should wait for a helper to
    /// arrive (handoff) before returning to a charging location. 
    bool m_handoff{true};

    bool m_proactive{false};

    double m_currentTime{0.0};

    double m_chargingRate{5.0};

    std::unordered_map<Agent*, BatteryBreak> m_batteryBreaks;

    /// Map agents to their current knowledge of other agents' plan versions.
    std::unordered_map<Agent*, std::unordered_map<PlanningAgent*, size_t>> m_versionMap;
    ///@}

};

#endif
