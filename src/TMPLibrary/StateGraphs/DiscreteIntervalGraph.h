#ifndef DISCRETE_INTERVAL_GRAPH_H_
#define DISCRETE_INTERVAL_GRAPH_H_

#include "Behaviors/Agents/HandoffAgent.h"

#include "ConfigurationSpace/RoadmapGraph.h"

#include "MPLibrary/MPSolution.h"

#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"
#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/WholeTask.h"

#include <iostream>




class DiscreteIntervalGraph : public CombinedRoadmap {
  public:

    ///@name Local Types
    ///@{

    struct AvailableNode {
      size_t           m_vid;
      double           m_availableTime;
      OccupiedInterval m_nextAssignment;

      bool operator==(const AvailableNode _node) const {
        return (m_vid == _node.m_vid
            && m_availableTime == _node.m_availableTime
            && m_nextAssignment == _node.m_nextAssignment);
      }
    };

    typedef RoadmapGraph<CfgType, WeightType> TaskGraph;
    typedef RoadmapGraph<CfgType, WeightType> AvailableIntervalGraph;
		typedef std::unordered_map<HandoffAgent*,std::vector<std::pair<size_t,std::pair<size_t,size_t>>>> ConstraintMap;


    ///@}

    ///@name Construction
    ///@{

    DiscreteIntervalGraph();

    DiscreteIntervalGraph(XMLNode& _node);

    virtual ~DiscreteIntervalGraph() = default;

    ///@}
    ///@name Initialization
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name Accessors
    ///@{

    virtual TaskGraph* GetGraph() override;

    std::shared_ptr<AvailableIntervalGraph> GetAvailableIntervalGraph();

    std::pair<Agent*,std::pair<size_t,size_t>> GetInterval(size_t _vid);

    size_t ValidTransition(size_t _source, size_t _target, size_t _edge,
        size_t _sourceDistance, std::pair<size_t,Cfg> _update,
				ConstraintMap _constraints);

    //TODO::move some of this down to the StateGraphLevel with appropriate inherited variations
    /// @param _vid1 VID of start vertex in higher level graph
    /// @param _vid2 VID of goal vertex in higher level graph
    /// @param _forceMatch will check if one of the vids corresponds to a virtual node and will
    //				 set the robot types of the cfgs to match the other node.
    /// Finds the weight of the path between two cfgs in the
    /// in the lower level graph.
    size_t ExtractPathWeight(size_t _vid1, size_t _vid2, bool _forceMatch=false,
														 ConstraintMap _constraints = ConstraintMap(), 
														 size_t _startTime = 0, size_t _minEndTime = 0);

    std::vector<size_t> ExtractPath(size_t _vid1, size_t _vid2, bool _forceMatch=false, 
																		ConstraintMap _constraints = ConstraintMap(), 
																		size_t _startTime = 0, size_t _minEndTime = 0);

    /// @param _wholeTask is the task being appended to the high level graph in order
    //				 to be planned next.
    //	@return Returns the start and goal VIDs in the high level graph for the input
    //					task.
    std::pair<size_t,size_t> AddTaskToGraph(WholeTask* _wholeTask,
                                            std::set<size_t> _validAigVIDs={});

    void RemoveTaskFromGraph(WholeTask* _wholeTask);

    ///@}
    ///@name Helpers
    ///@{

    /// @param _start Cfg of start vertex in lowerer level graph
    /// @param _goal Cfg of goal vertex in lowerer level graph
    /// Finds the weight of the path between two cfgs in the
    /// in the lower level graph.
    size_t LowLevelGraphPathWeight(Cfg _start, Cfg _goal, ConstraintMap _constraints = ConstraintMap(),
																	 size_t _startTime = 0, size_t _minEndTime = 0);

    std::vector<size_t> LowLevelGraphPath(Cfg _start, Cfg _goal, ConstraintMap _constraints = ConstraintMap(),
																	 size_t _startTime = 0, size_t _minEndTime = 0);
    std::vector<size_t> LowLevelGraphPath(HandoffAgent* _agent, size_t _start, size_t _goal, 
																	 ConstraintMap _constraints = ConstraintMap(),
																	 size_t _startTime = 0, size_t _minEndTime = 0);
    //std::unordered_map<Agent*,std::vector<std::pair<double,OccupiedInterval>>>
    //  GetAgentAvailableIntervals(size_t _source, size_t _target);//, std::unordered_map<Agent*,
    //std::list<OccupiedInterval>> _RATCache);

    ///@}
    ///@name Debug
    ///@{

    void PrintGraph();

    void PrintAvailabilityGraph();

    std::pair<std::vector<size_t>,std::vector<size_t>> UpdateAvailableIntervalConstraint(
            HandoffAgent* _agent, size_t _startTime, size_t _endTime, size_t _startVID, size_t _endVID,
						WholeTask* _wholeTask, std::set<size_t> _validVIDs, ConstraintMap _constraints);

		std::pair<std::vector<size_t>,std::vector<size_t>> UpdateMotionConstraint(
											HandoffAgent* _agent,WholeTask* _wholeTask,
											std::set<size_t> _validVIDs, ConstraintMap _constraints);
 
    std::unordered_map<WholeTask*,std::vector<size_t>>& GetTaskAigVIDs();
    ///@}

  protected:

    ///@name Construction Helpers
    ///@{

    virtual void ConstructGraph() override;

    void CreateHighLevelGraph();

    void CreateAvailableIntervalGraph();

    bool ValidIntervalEdge(size_t _source, size_t _target, size_t _edge);

    void AddTaskToAvailableIntervalGraph(WholeTask* _wholeTask);

    ///@}
    ///@name member variables
    ///@{

    std::shared_ptr<TaskGraph> m_highLevelGraph{nullptr};

    std::shared_ptr<AvailableIntervalGraph> m_availableIntervalGraph{nullptr};

    std::unordered_map<string,std::vector<size_t>> m_deliveringVIDs;

    std::unordered_map<string,std::vector<size_t>> m_receivingVIDs;

    std::set<size_t> m_mainDelivering;
    std::set<size_t> m_goalDelivering;

	   //std::unordered_map<string,std::vector<size_t>> m_virtualVIDs;

    std::set<size_t> m_virtualVIDs;

    std::vector<size_t> m_currentTaskVIDs;

    std::unordered_map<size_t,
                       std::pair<Agent*,std::pair<size_t,size_t>>>
                       m_agentAvailableIntervalMap;

    std::unordered_map<size_t, AvailableNode> m_availableIntervalMap;

    /// Maps vids inside high level graph to the various nodes in the available
    /// interval graph with the same cfg
    std::unordered_map<size_t,std::vector<size_t>> m_intervalMap;
    std::unordered_map<size_t,size_t> m_parentIntervalMap;


    WholeTask* m_currentTask{nullptr};
    std::unordered_map<WholeTask*,std::vector<size_t>> m_hlgTaskVIDs;
    std::unordered_map<WholeTask*,std::vector<size_t>> m_aigTaskVIDs;
    ///@}
    //@name New Stuff 
    ///@{

		//Maps from source to target to path
		std::unordered_map<size_t,std::unordered_map<size_t,std::vector<size_t>>> m_highLevelEdgePaths;	
		std::unordered_map<size_t,std::unordered_map<size_t,std::vector<size_t>>> m_availableIntervalEdgePaths;	

    ///@}

};

/*----------------------------------------------------------------------------*/

#endif
