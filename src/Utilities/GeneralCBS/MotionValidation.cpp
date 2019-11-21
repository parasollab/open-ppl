#include "MotionValidation.h"

MotionValidation::
MotionValidation() {}	

MotionValidation::
MotionValidation(MPLibrary* _library, LowLevelSearch* _lowLevel) : Validation(_library,_lowLevel) {}	

MotionValidation::
~MotionValidation() {}

bool
MotionValidation::
InitialPlan(Decomposition* _decomposition, GeneralCBSTree& _tree) {
	return true;
}

bool 
MotionValidation::
ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) {

	auto constraints = FindMotionConflict(_node);
	
	if(constraints.first.m_time != MAX_DBL){
		AddMotionChildren(_node, _tree, constraints);
		return false;
	}

	return true;
}
	
std::pair<MotionConstraint,MotionConstraint>
MotionValidation::
FindMotionConflict(GeneralCBSNode& _node) {
	MotionConstraint one;
	MotionConstraint two;
	return std::make_pair(one,two);
}

void 
MotionValidation::
AddMotionChildren(GeneralCBSNode& _node, GeneralCBSTree& _tree, MotionConstraintPair _constraints) {
	
}
