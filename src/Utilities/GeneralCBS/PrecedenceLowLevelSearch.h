#ifndef PRECEDENCE_LOW_LEVEL_SEARCH_H_
#define PRECEDENCE_LOW_LEVEL_SEARCH_H_

#include "Utilities/GeneralCBS/TMPLowLevelSearch.h"

class PrecedenceLowLevelSearch : public TMPLowLevelSearch {
  public:

		///@name Local Types
		///@{

		///@}
		///@name Construction
		///@{

		PrecedenceLowLevelSearch();

		PrecedenceLowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel, 
													std::string _vcLabel, bool _debug = false); 

		///@}
		///@name Interface
		///@{

		///@input _node contains the solution we are trying to update
		///@input _task is the task which has a new constraint and needs updating
		///@output bool indicating if there is a valid plan for the task being updated
		virtual bool UpdateSolution(GeneralCBSNode& _node, SemanticTask* _task) override;
	
		//@}

  protected:	

		///@name Helper Functions
		///@{

		void ClearFlowSubtree(GeneralCBSNode& _node, size_t _vid);

		void UpdateCompletionTimes(GeneralCBSNode& _node);

		bool PlanSubtree(GeneralCBSNode& _node, size_t _vid);

		bool ExtractPlan(GeneralCBSNode& _node);

		double ComputeStartTime(GeneralCBSNode& _node, size_t _vid);

		double EvaluateFunction(TBDFunction _func);

		bool UpdateTaskGroup(GeneralCBSNode& _node, std::vector<size_t> _taskGroup, size_t _parentVID,
						double _precedence, std::unordered_set<size_t>& _seen, 
						std::priority_queue<std::pair<double,size_t>>& _pq);

		bool UpdateIndividualTask(GeneralCBSNode& _node, SemanticTask* _task, size_t _vid, double _precedence);

		bool CheckSynchronization(GeneralCBSNode& _node, std::unordered_set<size_t> _tasks);
		///@}
		///@name Internal State
		///@{

		std::unordered_map<size_t,double> m_taskInitiationTimes;
		std::unordered_map<size_t,double> m_taskCompletionTimes;

		///@}

};

#endif
