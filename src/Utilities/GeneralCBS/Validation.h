#ifndef VALIDATION_H_
#define VALIDATION_H_

#include "MPLibrary/PMPL.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "Utilities/GeneralCBS/GeneralCBS.h"
#include "Utilities/GeneralCBS/LowLevelSearch.h"

class Validation {
  public:

		///@name Local Types
		///@{

		///@}
		///@name Construction
		///@{

		Validation();

		Validation(MPLibrary* _library, LowLevelSearch* _lowLevel);

		~Validation();

		///@}
		///@name Interface
		///@{

		///@input _decomposition Contatins the problem information
		///@input _tree CBS tree to which a root node with the initial plan is added
		///@output bool indicates if there is a valid plan for each simple task in the decomp
		virtual bool InitialPlan(Decomposition* _decomposition, GeneralCBSTree& _tree);

		///@input _node the CBS node for which the solution is being validated
		///@input _tree the CBS tree to which any child nodes will be added
		///@output bool indicates if the node's solution is valid
		virtual bool ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree);
	
		///@}

  protected:
		///@name Helper Functions
		///@{
		///@}
		///@name Internal State
		///@{
	
		MPLibrary* m_library;

		LowLevelSearch* m_lowLevel;

		///@}
};

#endif
