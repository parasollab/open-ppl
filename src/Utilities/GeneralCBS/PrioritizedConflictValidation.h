#ifndef PRIORITIZED_CONFLICT_VALIDATION_H_
#define PRIORITIZED_CONFLICT_VALIDATION_H_

#include "Validation.h"

class PrioritizedConflictValidation : public Validation {

	public :

		///@name Construction
		///@{

		PrioritizedConflictValidation(MPLibrary* _library, LowLevelSearch* _lowLevelSearch, 
																	TMPLibrary* _tmpLibrary, FindAllConflictsFunction _findAll);

		~PrioritizedConflictValidation();

		///@}
		///name@ Interface
		///@{

		virtual bool ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) override;

		///@}

	protected :

		///@name Helper Functions
		///@{
	
		std::pair<AllocationConstraint,AllocationConstraint> GetAllocationConstraints(
				std::pair<Assignment,Assignment>& _conflict);

		std::vector<GeneralCBSNode> CreateAllocationChildren(GeneralCBSNode& _node, 
			std::pair<AllocationConstraint,AllocationConstraint> _constraints);
		///@}
		///@name Internal State
		///@{

		FindAllConflictsFunction m_findAll;

		///@}
};

#endif
