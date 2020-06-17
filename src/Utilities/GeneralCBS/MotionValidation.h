#ifndef MOTION_VALIDATION_H_
#define MOTION_VALIDATION_H_

#include "Validation.h"
#include "TMPLibrary/TMPLibrary.h"

class MotionValidation : public Validation {
  public:

	///@name Local Types
	///@{

	typedef std::pair<MotionConstraint,MotionConstraint> MotionConstraintPair;

	///@}
	///@name Construction
	///@{

	MotionValidation();

	MotionValidation(MPLibrary* _library,LowLevelSearch* _lowLevel, 
									 TMPLibrary* _tmpLibrary, std::string _vcLabel, std::string _sgLabel);

	~MotionValidation();

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

	size_t CountConflicts(GeneralCBSNode& _node);	

	///@}

  private:

	///@name Helpers
	///@{

	///@input _node CBSNode containing the solution that is checked for conflict
	///@output if a conflict is found, return pair of motion constraints generated
	///			from this conflict
	///			otherwise, return a pair of empty constraints
	MotionConstraintPair FindMotionConflict(GeneralCBSNode& _node);

	///@input _node parent CBSNode
	///@input _tree CBSTree to add the children to
	///@input _constraints pair of motion constraints with which to spawn the children
	void AddMotionChildren(GeneralCBSNode& _node, GeneralCBSTree& _tree, MotionConstraintPair _constraints);
	
	void AvoidancePaths(GeneralCBSNode& _node);

	///@}
	///@name Internal State
	///@{

	///< State Graph Label
	std::string m_sgLabel;

	///< Validity Checker Label
	std::string m_vcLabel;

	bool m_debug{true};

	///@}

};

#endif
