#ifndef _PPL_CBS_EVALUATOR_H_
#define _PPL_CBS_EVALUATOR_H_

//////////////////////////////////////////////////////////////////
///
/// The intent is for this class to enable development of parallel
/// CBS. In the TMPLibrary we have access to the General CBS 
/// implementation in Utilites/GeneralCBS/ that should allow us to 
/// more rapidly create the parallel implementation. 
///
/// The next step is to merge this with the TMPCBS task evaluator
/// and use the 'fixed allocation' and 'decomposable' tags in 
/// SemanticTask to triggera normal CBS call without computing
/// decompositions or allocations.
///
//////////////////////////////////////////////////////////////////

#include "TaskEvaluatorMethod.h"

#include "TMPLibrary/Solution/TaskSolution.h"
#include "Utilities/GeneralCBS/GeneralCBS.h"

#include <string>

class CBSEvaluator : public TaskEvaluatorMethod {
	public:

		///@name Construction
		///@{

		CBSEvaluator();

		CBSEvaluator(XMLNode& _node);

		virtual ~CBSEvaluator();

		///@}
		///@name Evalautor Overrides
		///@{

		virtual bool Run(std::vector<WholeTask*> _wholeTasks = {}, std::shared_ptr<TaskPlan> _plan = nullptr) override;

		///@}

	private:

		///@name Helper Functions
		///@{

		void ConvertCBSSolutionToPlan(const CBSSolution& _solution);

		std::shared_ptr<TaskSolution> ConvertAssignmentToTaskSolution(Assignment& _assign);

		///@}
		///@name Internal State
		///@{
		
		std::string m_vcLabel;

		bool m_lowLevelDebug{false};

		bool m_parallel{false};

		///< Flag for post task dummy path to get out of the way
		bool m_avoidancePaths{true};
		///@}
};

#endif
