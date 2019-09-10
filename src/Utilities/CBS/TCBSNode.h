#ifndef TCBS_NODE_H_
#define TCBS_NODE_H_

#include "Behaviors/Agents/Coordinator.h"

#include "ConfigurationSpace/RoadmapGraph.h"

#include "TMPLibrary/TaskPlan.h"

#include "Utilities/CBS/NewCBSNode.h"
#include "Utilities/CBS/TaskConflict.h"


template <typename T, typename U>
class TCBSNode : public NewCBSNode<T, U> {
  public:

    ///@name Construction
    ///@{

		TCBSNode();
		
		TCBSNode(std::vector<WholeTask*> _tasks);

    TCBSNode(TCBSNode<HandoffAgent,std::pair<size_t,std::pair<size_t,size_t>>>* _parentNode, T* _toReplan);

		virtual ~TCBSNode();

    //@}
    ///@name Accessors
    ///@{

    double GetCost(bool _makespan = false) const override;
    size_t GetDiscreteCost(bool _makespan = false) const;

    ///@}
    ///@name Helpers
    ///@{

    void AddConflict(T* _t, NewConflict<U>* _c);

    size_t GetDepth();
    void SetDepth(size_t _depth);

		void AssignTask(WholeTask* _task, HandoffAgent* _agent);

		void AddAgentPath(HandoffAgent* _agent, std::vector<size_t> _path);

		std::set<WholeTask*> GetAssignedTasks();

		std::set<WholeTask*> GetUnassignedTasks();

		std::vector<WholeTask*> GetAgentAllocations(HandoffAgent* _agent);

		void DeallocateTask(HandoffAgent* _agent, WholeTask* _task);

		std::vector<std::vector<size_t>> GetAgentPaths(HandoffAgent* _agent);

		std::vector<size_t> GetAgentEntirePath(HandoffAgent* _agent);

		void ClearPaths(HandoffAgent* _agent);
    ///@}

  private:

    ///@name Internal State
    ///@{

    size_t m_depth{0};

		std::unordered_map<WholeTask*,HandoffAgent*> m_taskAllocation;

		std::unordered_map<HandoffAgent*,std::vector<WholeTask*>> m_agentAllocations;

		std::unordered_map<HandoffAgent*,std::vector<std::vector<size_t>>> m_agentPaths;

		std::unordered_map<HandoffAgent*, std::vector<size_t>> m_agentEntirePaths;
    ///@}

};

/*------------------------------- Construction -------------------------------*/

template<typename T, typename U>
TCBSNode<T, U>::
TCBSNode() { 
	this->m_conflicts = new std::unordered_map<T*, std::list<NewConflict<U>*>>();
}

template<typename T, typename U>
TCBSNode<T, U>::
TCBSNode(std::vector<WholeTask*> _tasks) { 
	for(auto task : _tasks) {
		m_taskAllocation[task] = nullptr;
	}
	this->m_conflicts = new std::unordered_map<T*, std::list<NewConflict<U>*>>();
}

template<typename T, typename U>
TCBSNode<T, U>::
TCBSNode(TCBSNode<HandoffAgent,std::pair<size_t,std::pair<size_t,size_t>>>* _parentNode, T* _toReplan){
	this->m_toReplan = _toReplan;

	m_taskAllocation = _parentNode->m_taskAllocation;
	m_agentAllocations = _parentNode->m_agentAllocations;
	m_agentPaths = _parentNode->m_agentPaths;
	m_agentEntirePaths = _parentNode->m_agentEntirePaths;
	m_depth = _parentNode->m_depth+1;

	this->m_conflicts = new std::unordered_map<T*, std::list<NewConflict<U>*>>();
	*(this->m_conflicts) = *(_parentNode->m_conflicts);
}


template <typename T, typename U>
TCBSNode<T,U>::
~TCBSNode(){}
/*-------------------------------- Accessors ---------------------------------*/

template <typename T, typename U>
double
TCBSNode<T, U>::
GetCost(bool _makespan) const{
  /*double sum = 0;
	double max = 0;
	for(auto agentPath : m_agentEntirePaths) {
		double length = agentPath.second.size();
		sum += length;
		if(length > max) {
			max = length;
		}
	}

	if(_makespan)
		return max;
	return sum;*/
	return double(GetDiscreteCost(_makespan));
}

template <typename T, typename U>
size_t
TCBSNode<T, U>::
GetDiscreteCost(bool _makespan) const{
  size_t sum = 0;
	size_t max = 0;
	/*
	for(auto agentPath : m_agentEntirePaths) {
		size_t length = agentPath.second.size();
		sum += length;
		if(length > max) {
			max = length;
		}
	}*/
	for(auto agentPaths : m_agentPaths) {
		size_t time = 0;
		for(auto path : agentPaths.second) {
			time += (path.size() -1);
			if(time > max)
				max = time;
			sum += time;
		}
	}
	if(_makespan)
		return max;
	return sum;
}

template <typename T, typename U>
void 
TCBSNode<T,U>::
AssignTask(WholeTask* _task, HandoffAgent* _agent) {
	m_taskAllocation[_task] = _agent;
	m_agentAllocations[_agent].push_back(_task);
}


template <typename T, typename U>
void
TCBSNode<T,U>::
AddAgentPath(HandoffAgent* _agent, std::vector<size_t> _path) {
	m_agentPaths[_agent].push_back(_path);
	for(auto vid : _path) {
		m_agentEntirePaths[_agent].push_back(vid);
	}
}

template <typename T, typename U>
std::set<WholeTask*> 
TCBSNode<T,U>::
GetAssignedTasks() {
	std::set<WholeTask*> assigned;
	for(auto allocation : m_taskAllocation) {
		if(allocation.second)
			assigned.insert(allocation.first);
	}
	return assigned;
}

template <typename T, typename U>
std::set<WholeTask*> 
TCBSNode<T,U>::
GetUnassignedTasks() {
	std::set<WholeTask*> assigned;
	for(auto allocation : m_taskAllocation) {
		if(!allocation.second)
			assigned.insert(allocation.first);
	}
	return assigned;
}

template <typename T, typename U>
std::vector<WholeTask*> 
TCBSNode<T,U>::
GetAgentAllocations(HandoffAgent* _agent) {
	return m_agentAllocations[_agent];
}

template <typename T, typename U>
std::vector<std::vector<size_t>> 
TCBSNode<T,U>::
GetAgentPaths(HandoffAgent* _agent) {
	return m_agentPaths[_agent];
}

template <typename T, typename U>
std::vector<size_t>
TCBSNode<T,U>::
GetAgentEntirePath(HandoffAgent* _agent) {
	return m_agentEntirePaths[_agent];
}

template <typename T, typename U>
void 
TCBSNode<T,U>::
DeallocateTask(HandoffAgent* _agent, WholeTask* _task) {
	std::remove(m_agentAllocations[_agent].begin(), 
							m_agentAllocations[_agent].end(),
							_task);
	m_taskAllocation[_task] = nullptr;
}

template <typename T, typename U>
void 
TCBSNode<T,U>::
ClearPaths(HandoffAgent* _agent) {
	m_agentPaths[_agent] = {};
	m_agentEntirePaths[_agent] = {};
}
/*-------------------------------- Helpers -----------------------------------*/
template<typename T, typename U>
void
TCBSNode<T, U>::
AddConflict(T* _t, NewConflict<U>* _c){
  std::list<NewConflict<U>*> current = (*(this->m_conflicts))[_t];
  current.push_back(_c);
  current.sort();
  (*(this->m_conflicts))[_t] = current;
}

template<typename T, typename U>
size_t
TCBSNode<T, U>::
GetDepth() {
  return m_depth;
}

template<typename T, typename U>
void
TCBSNode<T, U>::
SetDepth(size_t _depth) {
  m_depth = _depth;
}

#endif
