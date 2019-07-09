#ifndef MULTI_AGENT_DIJKSTRA_H_
#define MULTI_AGENT_DIJKSTRA_H_

#include <list>
#include <unordered_map>

#include "TMPLibrary/TMPBaseObject.h"
#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/WholeTask.h"

class MultiAgentDijkstra : public TMPBaseObject {
	public:

    typedef RoadmapGraph<Cfg, DefaultWeight<Cfg>> TaskGraph;
		
		///@name Constructor
		///@{

		MultiAgentDijkstra();
		
		MultiAgentDijkstra(XMLNode& _node);

		~MultiAgentDijkstra();

		///@}
		///@name Call method
		///@{

		bool Run(WholeTask* _wholeTask, TaskPlan* _plan = nullptr);	

		///@}
	private:

		///@name Helper Functions
		///@{
		

		void ExtractTaskPlan(const std::vector<size_t>& _path, WholeTask* _wholeTask, 
								std::unordered_map<size_t,double> _distance);
	
		std::shared_ptr<MPTask> CreateMPTask(Robot* _robot, Cfg _start, Cfg _goal);
    
		double MAMTPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance, size_t _goal);

    ///@}
    ///@name member variables
    ///@{

		std::string m_sgLabel;

		// Keeps track of the robot assigned to each node during the dijkstra search
    std::unordered_map<size_t,Agent*> m_nodeAgentMap;

		std::unordered_map<size_t,double> m_incomingEdgeMap;
		std::unordered_map<size_t,size_t> m_parentMap; 

		// Keeps track of RAT changes during searches through the high level graph
		std::unordered_map<size_t,std::unordered_map<Agent*,std::list<OccupiedInterval*>>> m_nodeRATCache;
		// Probably shouldn't need this for handoff tasks because if an agent is the cheapest option 
		// to receive a task that it already passed off, it should have just kept it.
		
		///@}
};

#endif
