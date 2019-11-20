#ifndef ALLOCATION_VALIDATION_H_
#define ALLOCATION_VALIDATION_H_

#include "Validation.h"

#include "TMPLibrary/TMPLibrary.h"

class AllocationValidation : Validation {
  public:

	///@name Local Types
	///@{

	typedef std::pair<AllocationConstraint,AllocationConstraint> AllocationConstraintPair;

	///@}
	///@name Construction
	///@{

	AllocationValidation();

	AllocationValidation(MPLibrary* _library, LowLevelSearch* _lowLevelSearch);

	~AllocationValidation();

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

  private:

	///@name Helper Functions
	///@{

	///@input _node CBSNode containing the solution that is checked for conflict
	///@output if a conflict is found, return pair of allocation constraints 
				generated from this conflict
				otherwise, return a pair of empty constraints
	AllocationConstraintPair FindAllocationConflict(GeneralCBSNode& _node);

	///@input _node parent CBSNode
	///@input _tree CBSTree to add the children to
	///@input _constraints pair of allocation constraints with which to spawn the children
	void AddAllocationChildren(GeneralCBSNode& _node, GeneralCBSTree& _tree, 
								AllocationConstraintPair _constraints);

	///@}
	///@name Internal State
	///@{
	
	TMPLibrary* m_tmpLibrary;
	
	///@}

};

#endif
