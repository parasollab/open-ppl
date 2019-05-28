#ifndef MULTI_AGENT_MULTI_TASK_PLANNER_H_
#define MULTI_AGENT_MULTI_TASK_PLANNER_H_

#include <list>
#include <unordered_map>

#include "Behaviors/Agents/Agent.h"
#include "Behaviors/TMPStrategies/TaskPlan.h"
#include "Behaviors/TMPStrategies/TMPStrategyMethod.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "MPLibrary/PMPL.h"
#include "MPProblem/MPTask.h"



class MultiAgentMultiTaskPlanner : public TMPStrategyMethod {

  public:

    typedef RoadmapGraph<Cfg, DefaultWeight<Cfg>> TaskGraph;

    ///@name Construction
    ///@{

    MultiAgentMultiTaskPlanner() = default;

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
    virtual TaskPlan PlanTasks(MPLibrary* _library, vector<HandoffAgent*> _agents,
                               vector<std::shared_ptr<MPTask>> _tasks, 
                               Robot* _superRobot,
                               std::unordered_map<std::string, 
                               std::unique_ptr<PlacementMethod>>* _ITPlacementMethods = nullptr) 
                               override;
       
    ///@}
    ///@name TaskGrap Functions
    ///@{

    void CreateHighLevelGraph(Robot* _superRobot);

    TaskPlan MAMTDijkstra(WholeTask& _wholeTask);

    void AddTaskToGraph(WholeTask& _wholeTask, Robot* _superRobot);

    void RemoveTaskFromGraph(WholeTask& _wholeTask);

    ///@}
    ///@name RAT Functions
    ///@{

    void InitializeRAT();

    ///@}
    ///@name Helper Functions

    /// @param _vid1 VID of start vertex in higher level graph
    /// @param _vid2 VID of goal vertex in higher level graph
    /// Finds the weight of the path between two cfgs in the 
    /// in the lower level graph. 
    double ExtractPathWeight(size_t _vid1, size_t _vid2);

    /// @param _start Cfg of start vertex in lowerer level graph
    /// @param _goal Cfg of goal vertex in lowerer level graph
    /// Finds the weight of the path between two cfgs in the 
    /// in the lower level graph. 
    double LowLevelGraphPathWeight(Cfg _start, Cfg _goal);

    double MAMTPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    /// @param _ei is the edge representing the selection of the new robot
    /// @param _minAgent is input as a nullptr and used to return the new agent/robot
    ///        selected by the function
    /// @return Returns the time the new robot is ready to start the next subtask
    double RobotSelection(typename TaskGraph::adj_edge_iterator& _ei, (Agent*)& _minAgent);

    ///@}


  private:

    TaskGraph m_highLevelGraph;

    std::unordered_map<agent*,std::pair<Cfg,double>> m_RAT; ///< Robot Availability Table

    std::vector<TaskPlan*> m_taskPlans;

    std::unordered_map<string,std::vector<size_t>> m_deliveringVIDs;

    std::unordered_map<string,std::vector<size_t>> m_receivingVIDs;

    std::vector<size_t> m_currentTaskVIDs;

    // Keeps track of the robot assigned to each node during the dijkstra search
    std::unordered_map<size_t,agent*> m_nodeAgentMap; 
};

#endif
