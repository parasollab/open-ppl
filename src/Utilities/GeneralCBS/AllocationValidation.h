#ifndef ALLOCATION_VALIDATION_H_
#define ALLOCATION_VALIDATION_H_

#include "Validation.h"

#include "TMPLibrary/TMPLibrary.h"

class AllocationValidation : public Validation {
  public:

	///@name Local Types
	///@{

	typedef std::pair<AllocationConstraint,AllocationConstraint> AllocationConstraintPair;

	///@}
	///@name Construction
	///@{

	AllocationValidation();

	AllocationValidation(MPLibrary* _library, LowLevelSearch* _lowLevelSearch, TMPLibrary* _tmpLibrary);

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
	///			generated from this conflict
	///			otherwise, return a pair of empty constraints
	AllocationConstraintPair FindAllocationConflict(GeneralCBSNode& _node);

	///@input _node parent CBSNode
	///@input _tree CBSTree to add the children to
	///@input _constraints pair of allocation constraints with which to spawn the children
	void AddAllocationChildren(GeneralCBSNode& _node, GeneralCBSTree& _tree, 
								AllocationConstraintPair _constraints);

	///@input _a1 The assignment which the robot is completing before transitioning
	///@input _a2 The assignment which the robot is transitioning to start
	///@output bool indicating if the robot is able to make the transition before the 
	///					expected start of _a2. If feasible, the setup path of _a2 is updated.
	bool CanReach(Assignment& _a1, Assignment& _a2, GeneralCBSNode& _node);
	///@}
	///@name Internal State
	///@{
	
	TMPLibrary* m_tmpLibrary;
	
	///@}

};

#endif
