#ifndef COORDINATOR_H_
#define COORDINATOR_H_

#include "Agent.h"
#include "PlanningAgent.h"
#include "HandoffAgent.h"
#include "WholeTask.h"

#include "TMPLibrary/TMPLibrary.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"
#include "BatteryBreak.h"

//TODO remove dependency on these
#include "TMPLibrary/TMPStrategies/Actions/Action.h"

class HandoffAgent;
class InteractionTemplate;
class TaskPlan;
class TMPStrategyMethod;
class ITMethod;

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
///
/// None of the above information is accurate. Will and James wrote a bunch of
/// new stuff that may or (more likely) may not work.
////////////////////////////////////////////////////////////////////////////////
class Coordinator : public Agent {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef RoadmapGraph<CfgType, WeightType>         RoadmapType;
    typedef typename RoadmapType::vertex_descriptor   VID;
    typedef typename std::vector<VID>::const_iterator VIDIterator;

    ///@}
    ///@name Construction
    ///@{

    /// Create an agent group with some robot as the coordinator.
    /// @param _r The coordinator robot.
    Coordinator(Robot* const _r);

    Coordinator(Robot* const _r, XMLNode& _node);

    virtual ~Coordinator();

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

    /// Decide what to do when some group of agents are facing a potential
    /// collision. Send a command to each agent in this group to resolve the
    /// potential collision.
    void ArbitrateCollision();

    void CheckFinished();

    ///@}
    ///@name Accessors
    ///@{

    /// Gets the current time of the simulation.
    /// @return m_currentTime
    double GetCurrentTime();

    bool IsClearToMoveOn(HandoffAgent* _agent);

    HandoffAgent* GetCapabilityAgent(std::string _capability);

	TMPStrategyMethod* GetCurrentStrategy();

    std::unordered_map<std::shared_ptr<MPTask>,std::vector<Cfg>> m_interactionPathsDelivering;
    std::unordered_map<std::shared_ptr<MPTask>,std::vector<Cfg>> m_interactionPathsReceiving;
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

    /// Send a group member to a designated location by assigning a new task.
    /// The group member still needs to plan for the new task with PMPL.
    /// @param _member The group member to dispatch.
    /// @param _where The location to send _member.
    void DispatchTo(Agent* const _member, std::unique_ptr<Boundary>&& _where);

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

    ///@}
    ///@name Initialize Functions
    ///@{

    /// Makes sure all of the agents are ready to execute the plans
    void InitializeAgents();

    ///@}
    ///@name Helpers
    ///@{

    /// Converts the action plan returned by the TMP method into a task plan
    std::vector<std::shared_ptr<MPTask>> ConvertActionsToTasks
                    (std::vector<std::shared_ptr<Action>> _actionPlan);

    /// Assigns the tasks generated by the TMP action plan to the corresponding
    //robots
    void TMPAssignTasks(std::vector<std::shared_ptr<MPTask>> _taskPlan);

	void DistributeTaskPlan(TaskPlan* _taskPlan);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    TMPLibrary* m_tmpLibrary{nullptr};   ///< The shared-roadmap planning library.

    MPSolution* m_solution{nullptr}; ///< The shared-roadmap solution.

    //std::unique_ptr<Environment> m_handoffEnvironment;    ///< The handoff template environment.

    /// Used to keep IT roadmaps separate for visualization in simulator
    //std::unique_ptr<RoadmapType> m_handoffTemplateRoadmap;

    //std::vector<RoadmapType> m_translatedHandoffs; ///< Translated handoffs

    /// The VIDs of all individual agent roadmaps in each transformed handoff template.
    //std::vector<std::vector<size_t>> m_transformedRoadmaps;

    /// The VIDs of the start and end points of the whole tasks in the
    /// megaRoadmap
    //std::vector<std::vector<size_t>> m_wholeTaskStartEndPoints;

    //std::unordered_map<std::string, RoadmapType*> m_capabilityRoadmaps;

    //RoadmapType* m_megaRoadmap{nullptr}; ///< The combined roadmap of all heterogenous robots and handoffs.

    std::vector<std::string> m_memberLabels;  ///< Labels for the group members.
    std::vector<HandoffAgent*> m_memberAgents;       ///< All robots in the group.

    /// The relative priorities for member robots.
    std::unordered_map<Agent*, size_t> m_memberPriorities;

    /// The regular distance metric for finding nearest agents/chargers.
    std::string m_dmLabel;

    /// Current time in the simulator
    double m_currentTime{0.0};

    /// The list of WholeTasks, which need to be divided into subtasks
    //std::vector<WholeTask*> m_wholeTasks;

    /// The list of unassigned tasks, starting with all initial subtasks.
    //std::list<std::shared_ptr<MPTask>> m_unassignedTasks;

    /// Map agents to their current knowledge of other agents' plan versions.
    std::unordered_map<Agent*, std::unordered_map<PlanningAgent*, size_t>> m_versionMap;

    /// Map subtasks to the WholeTask that they are included in to access the
    /// next subtask.
    //std::unordered_map<std::shared_ptr<MPTask>, WholeTask*> m_subtaskMap;

    /// Maps agent capabilities to a dummy agent used for planning.
    //std::unordered_map<std::string, HandoffAgent*> m_dummyMap;

    /// Indicates which method is used for task decomposition and allocation
    //bool m_tmp{false};

    //bool m_it{true};
    /// Timer used for experiment timing
    nonstd::timer m_clock;

    /// List of simulator visualization graph ids
    std::vector<size_t> m_simulatorGraphIDs;

    /// Tempoary Map of IT Placement Method Options to use during development
    //std::unordered_map<std::string, std::unique_ptr<PlacementMethod>> m_ITPlacementMethods;

    //double m_connectionThreshold{1.5};

	//TODO:: make this more formal and not specifically ITMethod
	ITMethod* m_tmpStrategy;
    //std::string m_tmpStrategy;

    ///@}

};

#endif
