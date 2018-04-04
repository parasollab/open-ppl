#ifndef PLANNING_AGENT_H_
#define PLANNING_AGENT_H_

#include "Agent.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"

#include <memory>


////////////////////////////////////////////////////////////////////////////////
/// Abstract base class for agents which will plan paths using PMPL.
////////////////////////////////////////////////////////////////////////////////
class PlanningAgent : public Agent {

  public:

    ///@name Construction
    ///@{

    PlanningAgent(Robot* const _r);

    PlanningAgent(Robot* const _r, const PlanningAgent& _a);

    PlanningAgent(Robot* const _r, XMLNode& _node);

    virtual ~PlanningAgent();

    ///@}
    ///@name Agent Interface
    ///@{

    virtual void Initialize() override;

    virtual void Step(const double _dt) override;

    virtual void Uninitialize() override;

    virtual void SetTask(std::shared_ptr<MPTask> const _task) override;

    ///@}
    ///@name Planning
    ///@{

    /// Does the agent have a plan for its current task.
    /// @return True if the agent has a plan.
    virtual bool HasPlan() const = 0;

    /// Clear the agent's current plan. Overrides should always call this base
    /// method to update the plan version.
    virtual void ClearPlan();

    /// Is the agent currently generating a plan.
    /// @return True if the agent is generating a plan.
    bool IsPlanning() const;

    /// Get the current version number for the agent's plan.
    /// @return The plan version number.
    size_t GetPlanVersion() const;

    ///@}

  protected:

    ///@name Planning Helpers
    ///@{

    /// Generate a plan for the agent's current task.
    virtual void GeneratePlan();

    /// Function call for PMPL.
    virtual void WorkFunction(std::shared_ptr<MPProblem> _problem);

    ///@}
    ///@name Task Helpers
    ///@{

    /// Select the next task for this agent.
    /// @return True if a new task was selected, or false if none remain.
    virtual bool SelectTask();

    /// Evaluate the agent's progress on its current task.
    /// @return True if we should continue the current task, false otherwise.
    virtual bool EvaluateTask() = 0;

    /// Continue executing the agent's current task.
    /// @param _dt The step length.
    virtual void ExecuteTask(const double _dt) = 0;

    ///@}
    ///@name Internal State
    ///@{

    std::unique_ptr<MPLibrary> m_library;   ///< This agent's planning library.
    std::unique_ptr<MPSolution> m_solution; ///< The current solution.
    std::thread m_thread;                   ///< Thread for agent to run PMPL.
    std::atomic<bool> m_planning{false};    ///< Is the agent currently planning.
    size_t m_planVersion{1};                ///< The current plan version.

    ///@}

};

#endif
