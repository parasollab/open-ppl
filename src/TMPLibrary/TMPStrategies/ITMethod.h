#ifndef MULTI_AGENT_MULTI_TASK_PLANNER_H_
#define MULTI_AGENT_MULTI_TASK_PLANNER_H_

#include <list>
#include <unordered_map>

#include "Behaviors/Agents/Agent.h"

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/RoadmapGraph.h"

#include "MPLibrary/PMPL.h"

#include "MPProblem/MPTask.h"

#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"



class ITMethod : public TMPStrategyMethod {

  public:

    typedef RoadmapGraph<Cfg, DefaultWeight<Cfg>> TaskGraph;

    ///@name Construction
    ///@{

    ITMethod(XMLNode& _node);

		ITMethod(bool _useITs, bool _debug, std::string _dmLabel, double _connectionThreshold,
									Environment* _interactionEnvironment, 
									std::unordered_map<std::string, std::unique_ptr<ITPlacementMethod>>& _ITPlacementMethods);

    ITMethod() = default;

    ///@}
    ///@name Accessors
    ///@{

    ///@}
    ///@name Call Method
    ///@{

    /// Get plan for the input agents to perform the input tasks.
    /// _library needs to have the solution and problem set to the coordinator's
    /// values for these.
    virtual TaskPlan* PlanTasks(MPLibrary* _library, vector<HandoffAgent*> _agents,
                               vector<std::shared_ptr<MPTask>> _tasks) override;
       
    ///@}


  //private:
		///@name Combined Roadmap
    ///@{

		void QueryCombinedRoadmap();

    ///@}
		///@name Task Assignment
    ///@{

		virtual TaskPlan* AssignTasks() override;

		virtual void DecomposeTasks() override;

		std::shared_ptr<MPTask> AuctionTask(std::shared_ptr<MPTask> _nextTask, TaskPlan* _taskPlan);

    ///@}
		///@name Helper Functions
    ///@{

		void CopyRobotTypeRoadmaps();

        //Moved to task plan
		//std::shared_ptr<MPTask> GetNextSubtask(WholeTask* _wholeTask);

        //moved to task plan
		//void AddSubtask(std::shared_ptr<MPTask> _subtask);

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
