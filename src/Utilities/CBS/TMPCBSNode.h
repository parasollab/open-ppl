#ifndef TMPCBS_NODE_H_
#define TMPCBS_NODE_H_

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/HandoffAgent.h"

#include "ConfigurationSpace/RoadmapGraph.h"

#include "TMPLibrary/TaskPlan.h"

#include "Utilities/CBS/AgentAllocationConflict.h"
#include "Utilities/CBS/DiscreteAgentAllocation.h"
#include "Utilities/CBS/DiscreteMotionConflict.h"
#include "Utilities/CBS/NewCBSNode.h"

struct SubtaskPlan { 
	HandoffAgent* m_agent;
	std::vector<size_t> m_setupPath;
	size_t m_setupStartTime;
	std::vector<size_t> m_subtaskPath;
	size_t m_subtaskStartTime;
	size_t m_taskStartVID;
	size_t m_taskEndVID;
};


template <typename T, typename U>
class TMPCBSNode : public NewCBSNode<T, U> {
  public:

		///@local names
		///@{

		typedef std::pair<size_t,std::pair<size_t,size_t>>	 		 Motion;
		typedef DiscreteMotionConflict<Motion>						 			 MotionConflict;
		typedef AgentAllocationConflict<DiscreteAgentAllocation> TaskConflict;
		typedef std::unordered_map<HandoffAgent*,std::list<DiscreteAgentAllocation>> AgentAllocationMap;

		///@}
    ///@name Construction
    ///@{

		TMPCBSNode();
		
		TMPCBSNode(std::vector<WholeTask*> _tasks);

    TMPCBSNode(TMPCBSNode<WholeTask,Motion>* _parentNode, T* _toReplan);

		virtual ~TMPCBSNode();

    //@}
    ///@name Accessors
    ///@{

    double GetCost(bool _makespan = false) const override;

    size_t GetDiscreteCost(bool _makespan = false) const;

		void UpdateValidVIDs(std::vector<size_t> _invalids, std::vector<size_t> _valids, WholeTask* _wholeTask); 

    std::unordered_map<WholeTask*,std::set<size_t>>& GetValidVIDs();

		void SetValidVIDs(std::set<size_t> _validVIDs);

    void AddTaskConflict(T* _t, TaskConflict* _c);
	
		void AddMotionConflict(T* _t, HandoffAgent* _agent, MotionConflict* _c);

		std::unordered_map<HandoffAgent*,std::vector<Motion>>
		GetTaskMotionConstraints(T* _task);

		std::unordered_map<HandoffAgent*,std::vector<DiscreteAgentAllocation>>
		GetAgentAllocationConstraints(T* _task);

    size_t GetDepth();

    void SetDepth(size_t _depth);

		std::vector<SubtaskPlan>& GetTaskPlan(WholeTask* _task);

		void SetTaskPlan(WholeTask* _task, std::vector<SubtaskPlan> _plan);

		std::unordered_map<T*,std::vector<SubtaskPlan>>& GetTaskPlans();

		AgentAllocationMap CreateAllocations();

    ///@}

  private:

    ///@name Internal State
    ///@{

    size_t m_depth{0};

		std::unordered_map<T*,std::list<TaskConflict*>> m_taskConflicts;
		std::unordered_map<T*,std::unordered_map<HandoffAgent*,std::list<MotionConflict*>>> m_motionConflicts;

		std::unordered_map<T*,std::vector<SubtaskPlan>> m_taskPlans;

    std::unordered_map<WholeTask*,std::set<size_t>> m_validVIDs;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template<typename T, typename U>
TMPCBSNode<T, U>::
TMPCBSNode() { 
}

template<typename T, typename U>
TMPCBSNode<T, U>::
TMPCBSNode(std::vector<WholeTask*> _tasks) { 
}

template<typename T, typename U>
TMPCBSNode<T, U>::
TMPCBSNode(TMPCBSNode<WholeTask,Motion>* _parentNode, T* _toReplan){
	this->m_toReplan = _toReplan;

	m_depth = _parentNode->m_depth+1;
	
	m_taskConflicts = _parentNode->m_taskConflicts;
	m_motionConflicts = _parentNode->m_motionConflicts;

	m_taskPlans = _parentNode->m_taskPlans;

  m_validVIDs = _parentNode->m_validVIDs;
}


template <typename T, typename U>
TMPCBSNode<T,U>::
~TMPCBSNode(){}
/*-------------------------------- Accessors ---------------------------------*/

template <typename T, typename U>
double
TMPCBSNode<T, U>::
GetCost(bool _makespan) const{
  //double sum = 0;
	//double max = 0;

	//if(_makespan)
		//return max;
	//return sum;
	return double(GetDiscreteCost(_makespan));
}

template <typename T, typename U>
size_t
TMPCBSNode<T, U>::
GetDiscreteCost(bool _makespan) const{
  size_t sum = 0;
	size_t max = 0;

	for(auto taskPlan : m_taskPlans) {
		auto lastSubtask = taskPlan.second.back();
		size_t endTime = lastSubtask.m_subtaskStartTime + lastSubtask.m_subtaskPath.size();
		sum += endTime;
		if(max < endTime)
			max = endTime;
	}

	if(_makespan)
		return max;
	return sum;
}
/*-------------------------------- Helpers -----------------------------------*/
template<typename T, typename U>
void
TMPCBSNode<T, U>::
AddTaskConflict(T* _t, TaskConflict* _c){
	std::list<TaskConflict*>& current = m_taskConflicts[_t];

	for(auto iter = current.begin(); iter != current.end(); iter++) {
		if((*iter)->GetConstraint().m_startTime > _c->GetConstraint().m_startTime){
			current.insert(iter,_c);
			return;
		}
	}
	current.push_back(_c);
	//current.push_back(_c);
	//current.sort();
}

template<typename T, typename U>
void
TMPCBSNode<T, U>::
AddMotionConflict(T* _t, HandoffAgent* _agent, MotionConflict* _c){
	std::list<MotionConflict*>& current = m_motionConflicts[_t][_agent];

	for(auto iter = current.begin(); iter != current.end(); iter++) {
		if((*iter)->GetConstraint().first > _c->GetConstraint().first){
			current.insert(iter,_c);
			return;
		}
	}
	current.push_back(_c);
}

template<typename T, typename U>
size_t
TMPCBSNode<T, U>::
GetDepth() {
  return m_depth;
}

template<typename T, typename U>
void
TMPCBSNode<T, U>::
SetDepth(size_t _depth) {
  m_depth = _depth;
}

template<typename T, typename U>
std::unordered_map<WholeTask*,std::set<size_t>>&
TMPCBSNode<T,U>::
GetValidVIDs(){
  return m_validVIDs;
}

template<typename T, typename U>
void
TMPCBSNode<T,U>::
SetValidVIDs(std::set<size_t> _validVIDs) { 
	m_validVIDs = _validVIDs;
}

template<typename T, typename U>
void
TMPCBSNode<T,U>::
UpdateValidVIDs(std::vector<size_t> _invalids, std::vector<size_t> _valids, WholeTask* _wholeTask) {
  for(auto& invalid : _invalids) {
    m_validVIDs[_wholeTask].erase(invalid);
  }
  for(auto& valid : _valids) {
   	m_validVIDs[_wholeTask].insert(valid);
  }
}

template<typename T, typename U>
std::unordered_map<HandoffAgent*,std::vector<std::pair<size_t,std::pair<size_t,size_t>>>>
TMPCBSNode<T,U>::
GetTaskMotionConstraints(T* _task) {
	std::unordered_map<HandoffAgent*,std::vector<Motion>> constraintMap;

	for(auto agentConflicts : m_motionConflicts[_task]) {
		for(auto conflict : agentConflicts.second) {
			constraintMap[agentConflicts.first].push_back(conflict->GetConstraint());	
		}
	}
	return constraintMap;
}

template<typename T, typename U>
std::unordered_map<HandoffAgent*,std::vector<DiscreteAgentAllocation>>
TMPCBSNode<T,U>::
GetAgentAllocationConstraints(T* _task) {
	std::unordered_map<HandoffAgent*,std::vector<DiscreteAgentAllocation>> constraintMap;
	
	for(auto allocationConflict : m_taskConflicts[_task]) {
		auto allocation = allocationConflict->GetConstraint();
		constraintMap[allocation.m_agent].push_back(allocation);
	}
	return constraintMap;
}


template<typename T, typename U>
std::vector<SubtaskPlan>& 
TMPCBSNode<T,U>::
GetTaskPlan(WholeTask* _task) {
	return m_taskPlans[_task];
}

template<typename T, typename U>
void 
TMPCBSNode<T,U>::
SetTaskPlan(WholeTask* _task, std::vector<SubtaskPlan> _plan) {
	m_taskPlans[_task] = _plan;
}
		
template<typename T, typename U>
std::unordered_map<T*,std::vector<SubtaskPlan>>&
TMPCBSNode<T,U>::
GetTaskPlans() {
	return m_taskPlans;
}

template<typename T, typename U>
std::unordered_map<HandoffAgent*,std::list<DiscreteAgentAllocation>>
TMPCBSNode<T,U>::
CreateAllocations() {
	AgentAllocationMap allocs;

	for(auto taskConflicts : m_taskConflicts) {
		for(auto allocationConflict : taskConflicts.second) {
			auto allocation = allocationConflict->GetConstraint();
			allocs[allocation.m_agent].push_back(allocation);
		}
	}
	return allocs;
}
#endif
