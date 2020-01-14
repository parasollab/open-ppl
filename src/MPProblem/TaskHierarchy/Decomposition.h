#ifndef DECOMPOSITION_H_
#define DECOMPOSITION_H_

#include "SemanticTask.h"

class MPProblem;

class Decomposition {

	public:

		///@name Construction
		///@{

		Decomposition(std::shared_ptr<SemanticTask> _mainTask);
	
		Decomposition(XMLNode& _node, MPProblem* _problem);

		~Decomposition();

		///@}
		///@name Accessors
		///@{
		
		const std::string GetLabel() const;
	
		SemanticTask* GetMainTask();

		std::vector<SemanticTask*>& GetSimpleTasks();

		void AddTask(std::shared_ptr<SemanticTask> _task);

		SemanticTask* GetTask(std::string _label);

		void AddSimpleTask(SemanticTask* _task);

		///@}
		
	private:
		///@name Helper Functions
		///@{

		void ParseTask(XMLNode& _node, MPProblem* _problem);

		///@}
		///@name Internal State
		///@{

		std::unordered_map<std::string,std::shared_ptr<SemanticTask>> m_taskMap;

		///< Keeps track of smallest unit of decomposition
		std::vector<SemanticTask*> m_simpleTasks;

		///< highest task in the decomposition
		SemanticTask*	m_mainTask;

		std::string m_label;

		///@}
};
#endif
