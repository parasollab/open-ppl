#ifndef IT_METHOD_H_
#define IT_METHOD_H_

#include <list>
#include <unordered_map>

#include "Behaviors/Agents/Agent.h"

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/RoadmapGraph.h"

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"



class ITMethod : public TMPStrategyMethod {

  public:

    typedef RoadmapGraph<Cfg, DefaultWeight<Cfg>> TaskGraph;

    ///@name Construction
    ///@{

    ITMethod(XMLNode& _node);

		//ITMethod(bool _useITs, bool _debug, std::string _dmLabel, double _connectionThreshold,
		//							Environment* _interactionEnvironment);

    ITMethod() = default;

    ///@}
  
	protected:
	
		///@name Helper Functions
    ///@{
    
		/// Get plan for the input agents to perform the input tasks.
    /// _library needs to have the solution and problem set to the coordinator's
    /// values for these.
    virtual void PlanTasks() override;
       

		virtual void AssignTasks() override;

		virtual void DecomposeTasks() override;
		
		void QueryCombinedRoadmap();

		std::shared_ptr<MPTask> AuctionTask(std::shared_ptr<MPTask> _nextTask);

		void CopyRobotTypeRoadmaps();

    /// Inserts the subtask into the unassignedTasks list at the appropriate point
		void AddSubtask(std::shared_ptr<MPTask> _subtask);

    ///@}
    //@name Member Variables
    ///@{

		//TODO add task assignment as its own class (e.g. auction method)
		//TODO maybe put a base function in TMPStrategyMethod that can use it if desired

		//TODO this needs to be changed to a priority queue to allow for multiple tasks
		std::list<std::shared_ptr<MPTask>> m_unassignedTasks;
    
    ///@}

};

#endif
