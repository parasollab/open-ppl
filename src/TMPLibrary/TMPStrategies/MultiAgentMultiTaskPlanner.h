#ifndef MULTI_AGENT_MULTI_TASK_PLANNER_H_
#define MULTI_AGENT_MULTI_TASK_PLANNER_H_

#include <list>
#include <unordered_map>

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"

class WholeTask;

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
    
		TaskPlan* MAMTDijkstra(WholeTask* _wholeTask);

    double MAMTPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance);

    ///@}


    std::vector<TaskPlan*> m_taskPlans;

    // Keeps track of the robot assigned to each node during the dijkstra search
    std::unordered_map<size_t,HandoffAgent*> m_nodeAgentMap; 
};

#endif
