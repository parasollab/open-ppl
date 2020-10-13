#include "ComposeValidation.h"

#include "AllocationValidation.h"
#include "MotionValidation.h"

ComposeValidation::
ComposeValidation() {}	

ComposeValidation::
ComposeValidation(std::vector<Validation*> _validations) : m_validations(_validations) {}	

ComposeValidation::
~ComposeValidation() {}

bool
ComposeValidation::
InitialPlan(Decomposition* _decomposition, GeneralCBSTree& _tree) {
	return m_validations[0]->InitialPlan(_decomposition, _tree);
}

bool 
ComposeValidation::
ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) {
	for(auto valid : m_validations) {
		if(!valid->ValidatePlan(_node, _tree))
			return false;
	}

	return true;
}
