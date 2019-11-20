#include "GeneralCBS.h"

bool 
GeneralCBSNode::
operator>(GeneralCBSNode _other) {
	return m_cost > _other.m_cost;
}

CBSSolution
ConflictBasedSearch(Decomposition* _decomposition, size_t _numIterations, 
					InitialPlanFunction _initial, ValidationFunction _validation) {

	GeneralCBSTree tree;

	size_t counter = 0;

	_initial(_decomposition, tree);

	do {

		CBSNode current = tree.top();
		tree.pop();
	
		if(!_validation(current, tree))
			continue;

		return current.m_solution;

	} while(!tree.empty() and counter < _numIterations)

	return CBSSolution();
	
}
