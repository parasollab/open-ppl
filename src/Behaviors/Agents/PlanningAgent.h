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

    ///@}

  protected:

    ///@name Planning Helpers
    ///@{
    /// @TODO Make the plan generation happen concurrently with the simulation
    ///       instead of blocking it. The Step function should check if the plan
    ///       is complete before resuming.

    /// Generate a plan for the agent's current task.
    /// @return The resulting solution object.
    virtual void GeneratePlan();

    ///@}
    ///@name Task Helpers
    ///@{

    /// Select the next task for this agent.
    /// @return True if a new task was selected, or false if none remain.
    virtual bool SelectTask();

    /// Evaluate the agent's progress on its current task.
    /// @return True if the task is complete, false otherwise.
    virtual bool EvaluateTask() = 0;

    /// Continue executing the agent's current task.
    /// @param _dt The step length.
    virtual void ExecuteTask(const double _dt) = 0;

    ///@}
    ///@name Internal State
    ///@{

    std::unique_ptr<MPLibrary> m_library;   ///< This agent's planning library.
    std::unique_ptr<MPSolution> m_solution; ///< The current solution.

    ///@}

};

#endif
