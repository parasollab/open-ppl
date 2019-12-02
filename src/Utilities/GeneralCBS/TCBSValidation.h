#ifndef TCBS_VALIDATION_H_
#define TCBS_VALIDATION_H_

#include "Utilities/GeneralCBS/Validation.h"

#include "TMPLibrary/TMPLibrary.h"

class TCBSValidation : public Validation {
  public:

		///@name Local Types
		///@{

		///@}
		///@name Construction
		///@{

		TCBSValidation();

		TCBSValidation(MPLibrary* _library, LowLevelSearch* _lowLevel, TMPLibrary* _tmpLibrary);

		~TCBSValidation();

		///@}
		///@name Interface
		///@{

		///@input _decomposition Contatins the problem information
		///@input _tree CBS tree to which a root node with the initial plan is added
		///@output bool indicates if there is a valid plan for each simple task in the decomp
		virtual bool InitialPlan(Decomposition* _decomposition, GeneralCBSTree& _tree) override;

		///@input _node the CBS node for which the solution is being validated
		///@input _tree the CBS tree to which any child nodes will be added
		///@output bool indicates if the node's solution is valid
		virtual bool ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) override;
	
		///@}

  protected:
		///@name Helper Functions
		///@{
		void CreateAllocationChildren(GeneralCBSNode& _node, GeneralCBSTree& _tree, 
									std::vector<Assignment>& _unallocated, std::unordered_set<Agent*>& _occupied);
		///@}
		///@name Internal State
		///@{
	
		TMPLibrary* m_tmpLibrary;

		///@}
};

#endif
