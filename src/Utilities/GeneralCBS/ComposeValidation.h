#ifndef COMPOSE_VALIDATION_H_
#define COMPOSE_VALIDATION_H_

#include "Utilities/GeneralCBS/Validation.h"

class ComposeValidation : public Validation {
  public:

	///@name Local Types
	///@{

	///@}
	///@name Construction
	///@{

	ComposeValidation();

	ComposeValidation(std::vector<Validation*> _validations);

	~ComposeValidation();

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

	///@name Internal State
	///@{
	
	std::vector<Validation*> m_validations;

	///@}
};

#endif
