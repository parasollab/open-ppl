#ifndef IT_METHOD_H_
#define IT_METHOD_H_

#include <list>
#include <unordered_map>

#include "Behaviors/Agents/Agent.h"

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GenericStateGraph.h"

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"



class ITMethod : public TMPStrategyMethod {

  public:

    typedef GenericStateGraph<Cfg, DefaultWeight<Cfg>> TaskGraph;

    ///@name Construction
    ///@{
    ITMethod();

    ITMethod(XMLNode& _node);

		virtual ~ITMethod() = default;

    ///@}
  
	protected:
	
		///@name Overrides
    ///@{
    
    virtual void PlanTasks() override;
       
		virtual void AssignTasks() override;

		virtual void DecomposeTasks() override;
		
    ///@}
    ///@name Helper Functions
    ///@{

    /// Extract a motion path for the task from the 
    /// combined roadmap.
		void QueryCombinedRoadmap();

    ///@}
    //@name Member Variables
    ///@{

		// TODO::This needs to be changed to a priority queue to 
    // allow for multiple tasks
		std::list<std::shared_ptr<MPTask>> m_unassignedTasks;
    
    ///@}

};

#endif
