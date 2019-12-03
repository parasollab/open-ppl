#ifndef MP_LOW_LEVEL_SEARCH_H_
#define MP_LOW_LEVEL_SEARCH_H_

#include "LowLevelSearch.h"

#include "MPLibrary/PMPL.h"

class MPLowLevelSearch : public LowLevelSearch {
  public:

		///@name Construction
		///@{

		MPLowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel, std::string _vcLabel);

		///@}
		///@name Interface
		///@{

		///@input _node contains the solution we are trying to update
		///@input _task is the task which has a new constraint and needs updating
		///@output bool indicating if there is a valid plan for the task being updated
		virtual bool UpdateSolution(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task) override;
	
		//@}

  private:

		///@name Helper Functions
		///@{

		void ClearTaskPlan(Assignment& _assign, GeneralCBSNode& _node);

		void ClearAgentAssignments(Assignment& _assign, GeneralCBSNode& _node);

		bool PlanAssignments(GeneralCBSNode& _node);

		bool PlanAssignment(GeneralCBSNode& _node, Assignment& _assign, Assignment& _previous,
												double _startTime = 0, double _minEndTime = 0);

		///@}	
		///@name Internal State
		///@{

		MPLibrary* m_library;

		///@}

};

#endif
