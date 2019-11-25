#include "GeneralCBS.h"

GeneralCBSNode::
GeneralCBSNode(const CBSSolution _solution) {
	m_solution = _solution;
}

GeneralCBSNode::
GeneralCBSNode(const GeneralCBSNode& _parent) {
	m_solution = _parent.m_solution;
	m_motionConstraints = _parent.m_motionConstraints;
	m_allocationConstraints = _parent.m_allocationConstraints;
}

bool 
GeneralCBSNode::
operator>(const GeneralCBSNode& _other) const noexcept {
	return m_cost > _other.m_cost;
}

CBSSolution 
GeneralCBSNode::
GetSolution() const {
	return m_solution;
}
	
void 
GeneralCBSNode::
AddMotionConstraint(MotionConstraint& _c, Agent* _agent) {
	m_motionConstraints[_c.m_task][_agent].push_back(_c);	
}

const std::vector<MotionConstraint>&
GeneralCBSNode::
GetMotionConstraints(std::shared_ptr<SemanticTask> _task, Agent* _agent) {
	return m_motionConstraints[_task][_agent];
}

void 
GeneralCBSNode::
AddAllocationConstraint(AllocationConstraint& _c, Agent* _agent) {
	m_allocationConstraints[_c.m_task][_agent].push_back(_c);
}

const std::vector<AllocationConstraint>&
GeneralCBSNode::
GetAllocationConstraints(std::shared_ptr<SemanticTask> _task, Agent* _agent) {
	return m_allocationConstraints[_task][_agent];
}

void 
GeneralCBSNode::
UpdateTaskPlan(std::shared_ptr<SemanticTask> _task, std::vector<Assignment> _assignments) {
	//TODO::update solution from this input assignment
	
	m_solution.m_taskPlans[_task] = _assignments;

	for(auto& agentAssigns: m_solution.m_agentAssignments) {
		auto& assigns = agentAssigns.second;
		for(size_t i = 0; i < assigns.size(); i++) {
			if(assigns[i].m_task->GetParent() == _task) {
				assigns.erase(assigns.begin()+i);
				break;
			}
		}
	}

	for(auto assign : _assignments) {

		//std::vector<Assignment> temp = {assign};

		auto& agentAssigns = m_solution.m_agentAssignments[assign.m_agent];
		agentAssigns.push_back(assign);
		
		/*for(auto a : agentAssigns) {
			if(a.m_task->GetParent() != _task) {
				temp.push_back(a);
			}
		}*/

		//std::sort(temp.begin(),temp.end());
		//m_solution.m_agentAssignments[assign.m_agent] = temp;
		std::sort(agentAssigns.begin(),agentAssigns.end());
		//probably don't need in current structure
		m_solution.m_allocationMap[assign.m_task] = assign.m_agent;
	}

}

/*-------------------------- Conflict Based Search -------------------------*/

CBSSolution
ConflictBasedSearch(Decomposition* _decomposition, InitialPlanFunction _initial, 
										ValidationFunction _validation, size_t _numIterations) {

	GeneralCBSTree tree;

	size_t counter = 0;

	if(!_initial(_decomposition, tree))
		return CBSSolution();

	do {

		GeneralCBSNode current = tree.top();
		tree.pop();

		counter++;
	
		if(!_validation(current, tree))
			continue;

		return current.GetSolution();

	} while(!tree.empty() and counter < _numIterations);

	return CBSSolution();
	
}
	
