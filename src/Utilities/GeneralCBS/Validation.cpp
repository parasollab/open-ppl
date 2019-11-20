#include "Validation.h"

Validation::
Validation() {}	

Validation::
Validation(MPLibrary* _library, LowLevelSearch* _lowLevel) : m_library(_library) m_lowLevel(_lowLevel) {}	

Validation::
~Validation() {}

bool
Validation::
IntialPlan(Decomposition* _decomposition, LowLevelSearch* _lowLevel) {
	return true;
}

bool 
Validation::
ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree, LowLevelSearch* _lowLevel) {
	return true;
}
