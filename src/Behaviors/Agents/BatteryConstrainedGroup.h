#ifndef BATTERY_CONSTRAINED_GROUP_H_
#define BATTERY_CONSTRAINED_GROUP_H_

#include "Agent.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"


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

    enum Role {Worker, Helper, Charging};

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

    /// Decide what to do when some group of robots are facing a potential
    /// collision. Send a command to each robot in this group to resolve the
    /// potential collision.
    /// @param _robots The robots that are potentially in collision.
    void ArbitrateCollision(const vector<Robot*>& _robots);

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

    /// Change the role of a group member.
    /// @param _member The member agent.
    /// @param _role The new role for _member.
    void SetRole(Agent* const _member, const Role _role);

    /// Get the current role for a group member.
    /// @param _member The member agent.
    /// @return The current role of _member.
    Role GetRole(Agent* const _member);

    /// Get the set of all robots which are currently helpers.
    std::vector<Robot*>& GetHelpers();

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
    bool IsAtChargingStation(Agent* const _member);

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

    /// Map group members to their initial roles (needed for parsing only).
    std::unordered_map<std::string, Role> m_initialRoles;

    ///@}

};

#endif
