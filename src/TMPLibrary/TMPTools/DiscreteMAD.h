#ifndef DISCRETE_MAD_H_
#define DISCRETE_MAD_H_

#include <list>
#include <unordered_map>

#include "TMPLibrary/TMPBaseObject.h"
#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/WholeTask.h"

#include "Utilities/CBS/TMPCBSNode.h"

class DiscreteMAD : public TMPBaseObject {
  public:

    typedef RoadmapGraph<Cfg, DefaultWeight<Cfg>> TaskGraph;
    typedef RoadmapGraph<Cfg, DefaultWeight<Cfg>> AvailableIntervalGraph;
		typedef std::unordered_map<HandoffAgent*, std::list<DiscreteAgentAllocation>> AgentAllocationMap;

		typedef std::unordered_map<HandoffAgent*, std::vector<std::pair<size_t,std::pair<size_t,size_t>>>> 
						ConstraintMap;

		//typedef TMPCBSNode<WholeTask,std::pair<
		//				size_t,std::pair<size_t,size_t>>>::SubtaskPlan SubtaskPlan;

    ///@name Constructor
    ///@{

    DiscreteMAD();

    DiscreteMAD(XMLNode& _node);

    ~DiscreteMAD();

    ///@}
    ///@name Call method
    ///@{

    std::vector<SubtaskPlan> Run(WholeTask* _wholeTask, 
																 std::set<size_t> _validVIDs = {},
																 ConstraintMap _constraints = ConstraintMap());

    ///@}
  private:

    ///@name Helper Functions
    ///@{


    std::vector<SubtaskPlan> ExtractTaskPlan(const std::vector<size_t>& _path, WholeTask* _wholeTask,
        std::unordered_map<size_t,double> _distance, ConstraintMap _constraints = ConstraintMap());

    std::shared_ptr<MPTask> CreateMPTask(Robot* _robot, Cfg _start, Cfg _goal, WholeTask* _wholeTask);

    double AvailableIntervalPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
        size_t _sourceDistance, size_t _targetDistance, size_t start, size_t _goal,
				WholeTask* _task, ConstraintMap _constraints);

		SubtaskPlan CreateSubtaskPlan(HandoffAgent* _agent, size_t _start, size_t _end, 
																	size_t _startTime, size_t _endTime, ConstraintMap _constraints = ConstraintMap());

		void InitializeRobotUpdates(size_t _start);
    ///@}
    ///@name member variables
    ///@{

    std::string m_sgLabel;

    // Keeps track of the robot assigned to each node during the dijkstra search
    std::unordered_map<size_t,Agent*> m_nodeAgentMap;

    std::unordered_map<size_t,double> m_incomingEdgeMap;
    std::unordered_map<size_t,size_t> m_parentMap;

    std::unordered_map<size_t,double> m_extractionCostMap;

    //TODO:: find a way to use this to account for overage changes caused by
    //reusing a robot earlier in a plan found through the baackwards search
    //process!!!!!!!!!!!!!!!!!!! hacking rn instead w/ next variable
    // Keeps track of RAT changes during searches through the high level graph
    std::unordered_map<size_t,std::unordered_map<Agent*,std::list<OccupiedInterval>>> m_nodeRATCache;
    // Probably shouldn't need this for handoff tasks because if an agent is the cheapest option
    // to receive a task that it already passed off, it should have just kept it.

    std::unordered_map<size_t,set<HandoffAgent*>> m_usedAgents;

    //Tracks robot updates down different explorations of the availability graph
    std::unordered_map<size_t,std::unordered_map<
                       Agent*,std::pair<size_t,Cfg>>> m_robotUpdates;

    ///@}
};

#endif
