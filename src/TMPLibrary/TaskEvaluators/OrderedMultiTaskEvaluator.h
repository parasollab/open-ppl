#ifndef ORDERED_MULTI_TASK_EVALUATOR_H_
#define ORDERED_MULTI_TASK_EVALUATOR_H_

#include <list>
#include <unordered_map>

#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"

class OrderedMultiTaskEvaluator : public TaskEvaluatorMethod {
	public:
		
		///@name Constructor
		///@{

		OrderedMultiTaskEvaluator();
		
		OrderedMultiTaskEvaluator(XMLNode& _node);

		~OrderedMultiTaskEvaluator();

		///@}
		///@name Call method
		///@{

		///@}
	private:

		virtual bool Run(std::vector<WholeTask*> _wholeTasks = {}, TaskPlan* _plan = nullptr) override;	

		std::string m_madLabel;
};

#endif
