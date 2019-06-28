#ifndef MULTI_AGENT_MULTI_TASK_PLANNER_H_
#define MULTI_AGENT_MULTI_TASK_PLANNER_H_

#include <list>
#include <unordered_map>

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"

class WholeTask;
class MPTask;

class MultiAgentMultiTaskPlanner : public TMPStrategyMethod {

  public:
    ///@name Construction
    ///@{

		MultiAgentMultiTaskPlanner();

    MultiAgentMultiTaskPlanner(XMLNode& _node);

    ~MultiAgentMultiTaskPlanner();

		virtual void Initialize();

    ///@}
    ///@name Accessors
    ///@{

    ///@}
    ///@name Call Method
    ///@{

    /// Get plan for the input agents to perform the input tasks.
    /// _library needs to have the solution and problem set to the coordinator's
    /// values for these.
    virtual void PlanTasks() override;
       
    ///@}

  private:

    ///@name Helper Functions
    ///@{
    
    ///@}
		///@name member variables
		///@{

    std::vector<TaskPlan*> m_taskPlans;
		
		///@}
};

#endif
