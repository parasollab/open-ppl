#ifndef HANDOFF_AGENT_H_
#define HANDOFF_AGENT_H_

#include "PathFollowingAgent.h"

class NetbookInterface;


////////////////////////////////////////////////////////////////////////////////
/// This agent follows a set of tasks and executes the helper worker behavior.
////////////////////////////////////////////////////////////////////////////////
class HandoffAgent : public PathFollowingAgent {

  public:

    ///@name Construction
    ///@{

    HandoffAgent(Robot* const _r);

    HandoffAgent(Robot* const _r, XMLNode& _node);

    virtual ~HandoffAgent();

    ///@}
    ///@name Simulation Interface
    ///@{

    virtual void Initialize() override;

    void InitializeRoadmap();

    ///@}
    ///@name Child Interface
    ///@{

    ///Set m_parentAgent to the an agent of the same capability.
    ///@param _parent The parent agent.
    void SetParentAgent(Agent* const _parent);

    virtual bool IsChild() const noexcept override;

    ///@}

    void GenerateCost(std::shared_ptr<MPTask> const _task);

    double GetPotentialCost() const;

    double GetTaskTime() const;

    MPSolution* GetMPSolution();

    void SetRoadmapGraph(RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _graph);

    void AddSubtask(std::shared_ptr<MPTask> _task);

  protected:

    ///@}
    ///@name Planning Helpers
    ///@{

    virtual void WorkFunction(std::shared_ptr<MPProblem> _problem) override;

    ///@}
    ///@name Task Helpers
    ///@{

    virtual bool SelectTask() override;

    ///@}
    ///@name Controller Helpers
    ///@{

    virtual void ExecuteControls(const ControlSet& _c, const size_t _steps)
        override;

    ///@}
    ///@name Internal State
    ///@{

    /// The parent group to which this agent belongs.
    Agent* m_parentAgent{nullptr};

    double m_distance{0.0}; ///< The distance traveled since localizing.

    double pathTime{0.0};

    /// The path for the task that is currently being bid on.
    vector<Cfg> m_potentialPath;

    /// The cost of completing a potential task
    double m_potentialCost;

    /// Stores all of the subtasks that the agent has been assigned
    std::list<std::shared_ptr<MPTask>> m_queuedSubtasks;

    ///@}

};

#endif
