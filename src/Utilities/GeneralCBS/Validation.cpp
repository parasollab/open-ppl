#include "Validation.h"

Validation::
Validation() {}	

Validation::
Validation(MPLibrary* _library, LowLevelSearch* _lowLevel) : m_library(_library), m_lowLevel(_lowLevel) {}	

Validation::
~Validation() {}

bool
Validation::
InitialPlan(Decomposition* _decomposition, GeneralCBSTree& _tree) {
	return true;
}

bool 
Validation::
ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) {
	return true;
}

