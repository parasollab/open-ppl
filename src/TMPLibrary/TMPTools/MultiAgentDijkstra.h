#ifndef MULTI_AGENT_DIJKSTRA_H_
#define MULTI_AGENT_DIJKSTRA_H_

#include <list>
#include <unordered_map>

#include "TMPLibrary/TMPBaseObject.h"
#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/WholeTask.h"

class MultiAgentDijkstra : public TMPBaseObject {
  public:

    typedef GenericStateGraph<Cfg, DefaultWeight<Cfg>> TaskGraph;
    typedef GenericStateGraph<Cfg, DefaultWeight<Cfg>> AvailableIntervalGraph;

    ///@name Constructor
    ///@{

    MultiAgentDijkstra();

    MultiAgentDijkstra(XMLNode& _node);

    ~MultiAgentDijkstra();

    ///@}
    ///@name Call method
    ///@{

    bool Run(WholeTask* _wholeTask, std::shared_ptr<TaskPlan> _plan = nullptr, std::set<size_t> _validVIDs = {});

    ///@}
  private:

    ///@name Helper Functions
    ///@{


    void ExtractTaskPlan(const std::vector<size_t>& _path, WholeTask* _wholeTask,
        std::unordered_map<size_t,double> _distance);

    std::shared_ptr<MPTask> CreateMPTask(Robot* _robot, Cfg _start, Cfg _goal, WholeTask* _wholeTask);

    double MAMTPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance, size_t start, size_t _goal);

    double AvailableIntervalPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance, size_t start, size_t _goal,
				WholeTask* _task);

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

    double m_overage;

    //Tracks robot updates down different explorations of the availability graph
    std::unordered_map<size_t,std::unordered_map<
                       Agent*,std::pair<double,Cfg>>> m_robotUpdates;

    ///@}
};

#endif
