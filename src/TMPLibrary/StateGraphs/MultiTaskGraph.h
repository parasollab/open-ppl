#ifndef PMPL_MULTI_TASK_GRAPH_H_
#define PMPL_MULTI_TASK_GRAPH_H_

#include "Behaviors/Agents/HandoffAgent.h"

#include "ConfigurationSpace/RoadmapGraph.h"

#include "MPLibrary/MPSolution.h"

#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"
#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/WholeTask.h"

#include <iostream>

class MultiTaskGraph : public CombinedRoadmap {
  public:

    typedef RoadmapGraph<CfgType, WeightType> TaskGraph;

    ///@name Construction
    ///@{

    MultiTaskGraph();

    MultiTaskGraph(XMLNode& _node);

    virtual ~MultiTaskGraph() = default;

    ///@}
    ///@name Initialization
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name Accessors
    ///@{

    virtual TaskGraph* GetGraph() override;

    //TODO::move some of this down to the StateGraphLevel with appropriate inherited variations
    /// @param _vid1 VID of start vertex in higher level graph
    /// @param _vid2 VID of goal vertex in higher level graph
    /// @param _forceMatch will check if one of the vids corresponds to a virtual node and will  
    //				 set the robot types of the cfgs to match the other node.
    /// Finds the weight of the path between two cfgs in the 
    /// in the lower level graph. 
    double ExtractPathWeight(size_t _vid1, size_t _vid2, bool _forceMatch=false);

    /// @param _ei is the edge representing the selection of the new robot
    /// @param _minAgent is input as a nullptr and used to return the new agent/robot
    ///        selected by the function
    /// @param _RATCache holds any updates to the RAT along the path to this node
    /// @return Returns the time the new robot is ready to start the next subtask
    double RobotSelection(size_t _source, size_t _target, Agent** _minAgent,
        std::unordered_map<Agent*,std::list<OccupiedInterval>>& _RATCache,
        size_t parent, double _previousEdge, std::set<HandoffAgent*> _usedAgents);

    /// @param _wholeTask is the task being appended to the high level graph in order
    //				 to be planned next.
    //	@return Returns the start and goal VIDs in the high level graph for the input
    //					task.
    std::pair<size_t,size_t> AddTaskToGraph(WholeTask* _wholeTask);

    void RemoveTaskFromGraph(WholeTask* _wholeTask);

    ///@}
    ///@name Helpers
    ///@{

    /// @param _start Cfg of start vertex in lowerer level graph
    /// @param _goal Cfg of goal vertex in lowerer level graph
    /// Finds the weight of the path between two cfgs in the
    /// in the lower level graph.
    double LowLevelGraphPathWeight(Cfg _start, Cfg _goal);

		std::unordered_map<Agent*,std::vector<std::pair<double,OccupiedInterval>>>
		GetAgentAvailableIntervals(size_t _source, size_t _target, std::unordered_map<Agent*,
													std::list<OccupiedInterval>> _RATCache);

    ///@}

  protected:

    ///@name Construction Helpers
    ///@{

    virtual void ConstructGraph() override;

    void CreateHighLevelGraph();

    ///@}
    ///@name Debug
    ///@{

    void PrintGraph();

    ///@}
    ///@name member variables
    ///@{

    TaskGraph* m_highLevelGraph;

    std::unordered_map<string,std::vector<size_t>> m_deliveringVIDs;

    std::unordered_map<string,std::vector<size_t>> m_receivingVIDs;
    
		std::unordered_map<string,std::vector<size_t>> m_virtualVIDs;

    std::vector<size_t> m_currentTaskVIDs;

		std::unordered_map<size_t, std::unordered_map<
				Agent*,std::vector<std::pair<double,OccupiedInterval>>>> m_agentSafeIntervalMap;
    ///@}

};

/*----------------------------------------------------------------------------*/

#endif
