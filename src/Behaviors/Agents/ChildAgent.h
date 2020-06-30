#ifndef _PMPL_CHILD_AGENT_H_
#define _PMPL_CHILD_AGENT_H_

#include "PathFollowingAgent.h"

class Coordinator;

class ChildAgent : public PathFollowingAgent {
	public :
    ///@name Construction
    ///@{

    ChildAgent(Robot* const _r);

    ChildAgent(Robot* const _r, XMLNode& _node);

    virtual ~ChildAgent();

    ///@}
    ///@name Simulation Interface
    ///@{

    virtual void Initialize() override;

    virtual void Step(const double _dt) override;

    ///@}
    ///@name Child Interface
    ///@{

    Coordinator* GetCoordinator();

    ///Set m_parentAgent to the an agent of the same capability.
    ///@param _parent The parent agent.
    void SetCoordinator(Coordinator* const _parent);

		///@}
		///@name Accessors
		///@{

    /// Returns this agents m_solution pointer
    MPSolution* GetMPSolution();

		///@}
    ///@name Task Helpers
    ///@{

		virtual bool SelectTask() override;

    /// Evaluate the agent's progress on its current task.
    /// @return True if we should continue the current task, false otherwise.
    virtual bool EvaluateTask();

    /// Continue executing the agent's current task.
    /// @param _dt The step length.
    virtual void ExecuteTask(const double _dt) override;
		
		virtual void GeneratePlan() override;
		///@}
	private:

    ///@name Controller Helpers
    ///@{

    virtual void ExecuteControls(const ControlSet& _c, const size_t _steps) override;

		///@}
    ///@name Internal State
    ///@{

		/// The parent group to which this agent belongs.
    Coordinator* m_coordinator{nullptr};
	
		///@}

};

#endif
