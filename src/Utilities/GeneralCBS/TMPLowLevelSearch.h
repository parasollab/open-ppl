#ifndef TMP_LOW_LEVEL_SEARCH_H_
#define TMP_LOW_LEVEL_SEARCH_H_

#include "TMPLibrary/TMPLibrary.h"
#include "Utilites/GeneralCBS/GeneralCBS.h"

class TMPLowLevelSearch {
  public:

	///@name Construction
	///@{

	TMPLowLevelSearch(MPLibrary* _library);

	///@}
	///@name Interface
	///@{

	///@input _node contains the solution we are trying to update
	///@input _task is the task which has a new constraint and needs updating
	///@output bool indicating if there is a valid plan for the task being updated
	virtual bool UpdateSolution(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task) override;
	
	//@}

  private:
	
	///@name Internal State
	///@{

	TMPLibrary* m_tmpLibrary;

	///@}

};

#endif
