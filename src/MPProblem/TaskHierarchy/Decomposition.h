#ifndef DECOMPOSITION_H_
#define DECOMPOSITION_H_

#include "SemanticTask.h"

class Decomposition {

	public:

		///@name Construction
		///@{

		Decomposition(std::shared_ptr<SemanticTask> _mainTask);
	
		Decomposition(XMLNode& _node);

		~Decomposition();

		///@}
		///@name Accessors
		///@{
		
		std::shared_ptr<SemanticTask> GetMainTask();

		std::vector<std::shared_ptr<SemanticTask>>& GetSimpleTasks();

		void AddSimpleTask(std::shared_ptr<SemanticTask> _task);

		///@}
		
	private:

		///@name Internal State
		///@{

		///< Keeps track of smallest unit of decomposition
		std::vector<std::shared_ptr<SemanticTask>> m_simpleTasks;

		///< highest task in the decomposition
		std::shared_ptr<SemanticTask>		  m_mainTask;

		std::string m_label;

		///@}
};
#endif
