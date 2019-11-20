#include "AllocationValidation.h"

AllocationValidation::
AllocationValidation() {}	

AllocationValidation::
AllocationValidation(MPLibrary* _library, LowLevelSearch* _lowLevel, TMPLibrary* _tmpLibrary) : 
		Validation(_library, _lowLevel) 
		m_tmpLibrary(_tmpLibrary) {}	

AllocationValidation::
~AllocationValidation() {}

bool
AllocationValidation::
IntialPlan(Decomposition* _decomposition) {
	return true;
}

bool 
AllocationValidation::
ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) {
	auto constraints = FindAllocationConflict(_node);

	if(constraints.first.m_startTime != Max_DBL) {
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
