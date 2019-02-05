#ifndef HANDOFF_AGENT_H_
#define HANDOFF_AGENT_H_

#include "PathFollowingAgent.h"

class Coordinator;

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

    Coordinator* GetParentAgent();
    ///Set m_parentAgent to the an agent of the same capability.
    ///@param _parent The parent agent.
    void SetParentAgent(Coordinator* const _parent);

    virtual bool IsChild() const noexcept override;

    /// Generates cost of this agent perfomring the input task and
    /// stores it in m_cost
    /// @param _task The task that the cost is being generated for
    void GenerateCost(std::shared_ptr<MPTask> const _task);

    /// Returns the cost generated by GenerateCost and stored in
    /// m_cost
    double GetPotentialCost() const;

    /// Returns the projected time of completion of current path
    double GetTaskTime() const;

    /// Returns this agents m_solution pointer
    MPSolution* GetMPSolution();

    /// Sets the roadmap graph in the m_solution
    /// @param _graph roadmap graph to set in the m_solution
    void SetRoadmapGraph(RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _graph);

    /// Appends a subtask onto this agents task queue
    /// @param _task new subtask to add to the queue
    void AddSubtask(std::shared_ptr<MPTask> _task);

    /// Checks if the agent has reached the handoff
    bool ReachedHandoff();

    /// Returns this agent's task queue
    std::list<std::shared_ptr<MPTask>> GetQueuedSubtasks();

    /// Checks if this agent is performing a subtask, or a setup task
    bool IsPerformingSubtask();

    /// Indicates which type of task this agent is performing
    /// @param _performing New value for performing subtask
    void SetPerformingSubtask(bool _performing);

    /// Indicates if this agent should continue performing its current task or
    /// not.
    /// False means select a new task.
    /// True means continue performing currrent task.
    virtual bool EvaluateTask() override;

    /// Returns agent collision priority
    size_t GetPriority();

    /// Set this agent's collision priority
    /// @param _p New collision priority
    void SetPriority(size_t _p);

    /// Allows the agent to move on from an interaction
    /// @param _clear New value for m_clearToMove
    void SetClearToMove(bool _clear);
    ///@}
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
    Coordinator* m_parentAgent{nullptr};

    double m_distance{0.0}; ///< The distance traveled since localizing.

    double pathTime{0.0}; ///< Time needed to execute current plan

    /// The path for the task that is currently being bid on.
    vector<Cfg> m_potentialPath;

    /// The cost of completing a potential task
    double m_potentialCost;

    /// Stores all of the subtasks that the agent has been assigned
    std::list<std::shared_ptr<MPTask>> m_queuedSubtasks;

    bool m_performingSubtask{true}; ///< If agent is performing subtask or setup task

    size_t m_priority{0}; ///< Collision priority

    bool m_clearToMove{false}; ///< Clear to move on from interaction

    /// List of simulator visualization graph ids
    std::vector<size_t> m_simulatorGraphIDs;

    bool m_generatingCost{false}; ///< Flag so work function knows which query method to use.
    ///@}

};

#endif
