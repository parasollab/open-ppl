#include "AllocationValidation.h"

#include "Utilities/MPUtils.h"

AllocationValidation::
AllocationValidation() {}	

AllocationValidation::
AllocationValidation(MPLibrary* _library, LowLevelSearch* _lowLevel, TMPLibrary* _tmpLibrary) : 
		Validation(_library, _lowLevel), 
		m_tmpLibrary(_tmpLibrary) {}	

AllocationValidation::
~AllocationValidation() {}

bool
AllocationValidation::
InitialPlan(Decomposition* _decomposition, GeneralCBSTree& _tree) {

	CBSSolution solution;
	solution.m_decomposition = _decomposition;
	GeneralCBSNode node(solution);


	//temp for RA-L problem structure

	for(auto& semanticTask : _decomposition->GetSimpleTasks()) {
		m_lowLevel->UpdateSolution(node, semanticTask);	
	}

	_tree.push(node);
	return true;
}

bool 
AllocationValidation::
ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) {
	auto constraints = FindAllocationConflict(_node);

	if(constraints.first.m_startTime != MAX_DBL) {
		AddAllocationChildren(_node, _tree, constraints);
		return false;
	}

	return true;
}

std::pair<AllocationConstraint,AllocationConstraint>
AllocationValidation::
FindAllocationConflict(GeneralCBSNode& _node) {
	AllocationConstraint one;
	AllocationConstraint two;
	return std::make_pair(one,two);
}

void
AllocationValidation::
AddAllocationChildren(GeneralCBSNode& _node, GeneralCBSTree& _tree, AllocationConstraintPair _constraints) {

}
