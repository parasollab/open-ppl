#ifndef LOW_LEVEL_SEARCH_H_
#define LOW_LEVEL_SEARCH_H_

#include "Utilites/GeneralCBS/GeneralCBS.h"

class LowLevelSearch {
  public:

	///@name Construction
	///@{

	LowLevelSearch() = default;

	///@}
	///@name Interface
	///@{

	///@input _node contains the solution we are trying to update
	///@input _task is the task which has a new constraint and needs updating
	///@output bool indicating if there is a valid plan for the task being updated
	virtual bool UpdateSolution(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task);
	
	//@}

  private:	

};

#endif
