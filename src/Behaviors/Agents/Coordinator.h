#ifndef COORDINATOR_H_
#define COORDINATOR_H_

#include "Agent.h"
#include "ChildAgent.h"
#include "PlanningAgent.h"

#include "TMPLibrary/TMPLibrary.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"

#include "TMPLibrary/Solution/Plan.h"

class TMPStrategyMethod;

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

    void InitializePlanningComponents();

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

    TMPLibrary* GetTMPLibrary();

    void SetRoadmapGraph(RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _graph);

    std::vector<std::string> GetMemberLabels();

    std::vector<ChildAgent*> GetChildAgents();

    std::unordered_map<std::shared_ptr<MPTask>,std::vector<Cfg>> m_interactionPathsDelivering;
    std::unordered_map<std::shared_ptr<MPTask>,std::vector<Cfg>> m_interactionPathsReceiving;
    ///@}
  private:

    ///@}
    ///@name Member Management
    ///@{

    /// Send a group member to a designated location by assigning a new task.
    /// The group member still needs to plan for the new task with PMPL.
    /// @param _member The group member to dispatch.
    /// @param _where The location to send _member.
    void DispatchTo(Agent* const _member, std::unique_ptr<Boundary>&& _where);

    ///@}
    ///@name Initialize Functions
    ///@{

    /// Makes sure all of the agents are ready to execute the plans
    void InitializeAgents();

    ///@}
    ///@name Helpers
    ///@{

    /// Assigns the tasks generated by the TMP action plan to the corresponding
    //robots
    void DistributePlan(Plan* _plan);

    void GenerateRandomTasks();

    ///@}

  protected:

    ///@name Internal State
    ///@{

    MPLibrary* m_library{nullptr};   ///< The shared-roadmap planning library.
    TMPLibrary* m_tmpLibrary{nullptr};   ///< The task and motion planning library.

    MPSolution* m_solution{nullptr}; ///< The shared-roadmap solution.

    std::vector<std::string> m_memberLabels;  ///< Labels for the group members.

    std::vector<ChildAgent*> m_childAgents;       ///< All robots in the group.

    /// The regular distance metric for finding nearest agents/chargers.
    std::string m_dmLabel;

    /// Current time in the simulator
    double m_currentTime{0.0};

    /// List of simulator visualization graph ids
    std::vector<size_t> m_simulatorGraphIDs;

    std::shared_ptr<Plan> m_plan{nullptr};

    size_t m_numRandTasks;

    bool m_runSimulator{true};

    bool m_runDummies;

    ///@}

};

#endif
