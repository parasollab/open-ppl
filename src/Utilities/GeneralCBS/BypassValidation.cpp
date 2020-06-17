#include "BypassValidation.h"

/******************************** Construction ****************************/

BypassValidation::
BypassValidation(MPLibrary* _library, LowLevelSearch* _lowLevel, TMPLibrary* _tmpLibrary,
						ValidationFunction _valid, ConflictCountFunction _count, size_t _maxLeaves) 
								: Validation(_library,_lowLevel,_tmpLibrary),
									m_validation(_valid),
									m_count(_count),
									m_maxLeaves(_maxLeaves) {}

BypassValidation::
~BypassValidation() {}

/******************************** Interface ****************************/

bool 
BypassValidation::
ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) {
	
	GeneralCBSTree subtree;

	// Keep track of which children to add to tree if 
	std::vector<GeneralCBSNode> children;

	size_t leaves = 0;

	// Check if node contains a valid solution and if not, push children into subtree
	//if(m_validation(_node,subtree))
	//	return true;

	double minCost = _node.GetCost();

	m_count(_node);

	GeneralCBSNode current;
	subtree.push(_node);

	do {
		GeneralCBSTree tempTree;
		
		current = subtree.top();
		subtree.pop();

		if(m_validation(current,tempTree)) {
	
			// Check if this is the initial node 
			if(leaves == 0)
				return true;

			_node.SetSolution(current.GetSolutionRef());
			_tree.push(_node);
			return false;
		}

		while(!tempTree.empty()) {

			auto leaf = tempTree.top();
			tempTree.pop();

			if(leaf.GetCost() != minCost) {
				children.push_back(leaf);
				continue;
			}
	
			m_count(leaf);

			if(leaf.GetAllocationConflictCount() > _node.GetAllocationConflictCount()) {
				children.push_back(leaf);
				continue;
			}

			if(leaf.GetAllocationConflictCount() < _node.GetAllocationConflictCount() or
					leaf.GetMotionConflictCount() < _node.GetMotionConflictCount()) {

				_node.SetSolution(leaf.GetSolutionRef());
				_tree.push(_node);
				return false;
			}

			if(leaf.GetMotionConflictCount() > _node.GetMotionConflictCount()) {
				children.push_back(leaf);
				continue;
			}

			subtree.push(leaf);

		}

		leaves++;

	} while(!subtree.empty() and leaves < m_maxLeaves);

	while(!subtree.empty()) {
		current = subtree.top();
		subtree.pop();
		children.push_back(current);
	}

	for(auto node : children) {
		_tree.push(node);
	}

	return false;
}
