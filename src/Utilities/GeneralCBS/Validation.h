#ifndef VALIDATION_H_
#define VALIDATION_H_

#include "MPLibrary/MPLibrary.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "Utilities/GeneralCBS/GeneralCBS.h"
#include "Utilities/GeneralCBS/LowLevelSearch.h"

class Validation {
  public:

	///@name Construction
	///@{

	Validation();

	Validation(MPLibrary* _library, LowLevelPlanner* _lowLevel);

	~Validation();

	///@}
	///@name Interface
	///@{

	///@input _decomposition Contatins the problem information
	///@input _tree CBS tree to which a root node with the initial plan is added
	///@output bool indicates if there is a valid plan for each simple task in the decomp
	virtual bool InitialPlan(Decomposition* _decomposition, GeneralCBSTree& _tree, LowLevelSearch* _lowLevel);

	///@input _node the CBS node for which the solution is being validated
	///@input _tree the CBS tree to which any child nodes will be added
	///@output bool indicates if the node's solution is valid
	virtual bool ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree, LowLevelSearch* _lowLevel);
	
	///@}

  private:

	LowLevelPlanner* m_lowLevel;

	MPLibrary* _library;
};

#endif
