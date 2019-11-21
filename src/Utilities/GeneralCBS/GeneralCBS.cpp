#include "GeneralCBS.h"

GeneralCBSNode::
GeneralCBSNode(const CBSSolution _solution) {
	m_solution = _solution;
}

GeneralCBSNode::
GeneralCBSNode(const GeneralCBSNode& _parent, const CBSSolution _solution) {
	m_solution = _solution;
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

void 
GeneralCBSNode::
AddAllocationConstraint(AllocationConstraint& _c, Agent* _agent) {
	m_allocationConstraints[_c.m_task][_agent].push_back(_c);
}

void 
GeneralCBSNode::
UpdateTaskPlan(std::shared_ptr<SemanticTask> _task, std::vector<Assignment> _assignments) {
	//TODO::update solution from this input assignment
}

CBSSolution
ConflictBasedSearch(Decomposition* _decomposition, InitialPlanFunction _initial, 
										ValidationFunction _validation, size_t _numIterations) {

	GeneralCBSTree tree;

	size_t counter = 0;

	_initial(_decomposition, tree);

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
	
