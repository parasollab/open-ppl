#ifndef MULTI_AGENT_MULTI_TASK_PLANNER_H_
#define MULTI_AGENT_MULTI_TASK_PLANNER_H_

#include <list>
#include <unordered_map>

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"
#include "TMPLibrary/WholeTask.h"

class MultiAgentMultiTaskPlanner : public TMPStrategyMethod {

  public:

    typedef RoadmapGraph<Cfg, DefaultWeight<Cfg>> TaskGraph;

    ///@name Construction
    ///@{

    MultiAgentMultiTaskPlanner(XMLNode& _node);

    ~MultiAgentMultiTaskPlanner() = default;

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
    ///@name TaskGrap Functions
    ///@{

    void CreateHighLevelGraph();

    TaskPlan* MAMTDijkstra(WholeTask* _wholeTask);

    void AddTaskToGraph(WholeTask* _wholeTask);

    void RemoveTaskFromGraph(WholeTask* _wholeTask);

    ///@}
    ///@name Helper Functions

    double MAMTPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance);

    ///@}

  private:

    std::vector<TaskPlan*> m_taskPlans;

    // Keeps track of the robot assigned to each node during the dijkstra search
    std::unordered_map<size_t,HandoffAgent*> m_nodeAgentMap; 
};

#endif
