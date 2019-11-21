#ifndef MP_LOW_LEVEL_SEARCH_H_
#define MP_LOW_LEVEL_SEARCH_H_

#include "LowLevelSearch.h"

#include "MPLibrary/PMPL.h"

class MPLowLevelSearch : public LowLevelSearch {
  public:

	///@name Construction
	///@{

	MPLowLevelSearch(MPLibrary* _library);

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

	MPLibrary* m_library;

	///@}

};

#endif
