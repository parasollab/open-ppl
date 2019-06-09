#ifndef MULTI_AGENT_MULTI_TASK_PLANNER_H_
#define MULTI_AGENT_MULTI_TASK_PLANNER_H_

#include <list>
#include <unordered_map>

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"

class WholeTask;
class MPTask;

class MultiAgentMultiTaskPlanner : public TMPStrategyMethod {

  public:

    typedef RoadmapGraph<Cfg, DefaultWeight<Cfg>> TaskGraph;

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
    
		TaskPlan* MAMTDijkstra(WholeTask* _wholeTask, size_t start, size_t _goal);

    double MAMTPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance, size_t _goal);

		TaskPlan* ExtractTaskPlan(const std::vector<size_t>& _path, WholeTask* _wholeTask,
															std::unordered_map<size_t,double> _distanceMap);

		std::shared_ptr<MPTask> CreateMPTask(Robot* _robot, Cfg _start, Cfg _goal);
    ///@}

    std::vector<TaskPlan*> m_taskPlans;

    // Keeps track of the robot assigned to each node during the dijkstra search
    std::unordered_map<size_t,Agent*> m_nodeAgentMap; 

		// Keeps track of RAT changes during searches through the high level graph
		std::unordered_map<size_t,std::unordered_map<Agent*,std::pair<Cfg,double>>> m_nodeRATCache;
		// Probably shouldn't need this for handoff tasks because if an agent is the cheapest option 
		// to receive a task that it already passed off, it should have just kept it.
};

#endif
