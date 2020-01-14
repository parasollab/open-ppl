#ifndef BYPASS_VALIDATION_H_
#define BYPASS_VALIDATION_H_

#include "Utilities/GeneralCBS/Validation.h"

class BypassValidation : public Validation {
  public:

		///@name Local Types
		///@{

		///@}
		///@name Construction
		///@{

		BypassValidation();

		BypassValidation(MPLibrary* _library, LowLevelSearch* _lowLevel, TMPLibrary* _tmpLibrary,
									ValidationFunction _valid, ConflictCountFunction _count, size_t _maxLeaves=2);

		~BypassValidation();

		///@}
		///@name Interface
		///@{

		///@input _node the CBS node for which the solution is being validated
		///@input _tree the CBS tree to which any child nodes will be added
		///@output bool indicates if the node's solution is valid
		virtual bool ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) override;
	
		///@}

  protected:
		///@name Helper Functions
		///@{

		///@}
		///@name Internal State
		///@{
	
		ValidationFunction m_validation;

		ConflictCountFunction m_count;

		size_t m_maxLeaves;

		///@}
};

#endif
