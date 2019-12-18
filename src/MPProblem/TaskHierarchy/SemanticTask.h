#ifndef SEMANTIC_TASK_H_
#define SEMANTIC_TASK_H_

#include "MPProblem/MPTask.h"

#include <unordered_map>

class SemanticTask {

	public:
	
		///@name Local Types
		///@{

		enum SubtaskRelation {
			XOR,
			AND
		};

		enum DependencyType {
			Completion,
			Initiation,
			Synchronous,
			Asynchronous,
			None
		};

		typedef std::unordered_map<DependencyType, std::vector<std::shared_ptr<SemanticTask>>, 
															 std::hash<int>> DependencyMap;

		///@}
		///@name Construction
		///@{

		SemanticTask();

		SemanticTask(XMLNode& _node);

		SemanticTask(std::shared_ptr<SemanticTask> _parent, std::shared_ptr<MPTask> _simpleTask, 
								 bool _decomposable = true);

		~SemanticTask();

		///@}
		///@name Accessors
		///@{

		std::string GetLabel() const;

		///< Sets the dependencies of all the semantic tasks below this in the hierarchy
		std::vector<std::shared_ptr<SemanticTask>> SetDependencies();

		///< Indicates if this is a simple task or not
		bool IsDecomposable();

		std::shared_ptr<MPTask> GetMotionTask();

		std::shared_ptr<SemanticTask> GetParent();

		void SetParent(std::shared_ptr<SemanticTask> _parent);

		void AddDependency(std::shared_ptr<SemanticTask> _task, DependencyType _type);

		DependencyMap& GetDependencies();

		bool IsFixedAssignment();

		void AddSubtask(std::shared_ptr<SemanticTask> _task);

		std::vector<std::shared_ptr<SemanticTask>> GetSubtasks();

		///@}

	private:

		///@name Internal State
		///@{

		///< Label distinguishing this semantic task
		std::string m_label;

		///< Parent SemanticTask that includes this task in its decomposition
		std::shared_ptr<SemanticTask> m_parent{nullptr};

		///< Set of subtasks that make up this task's decomposition
		std::vector<std::shared_ptr<SemanticTask>> m_subtasks;

		///< Relationship between the subtasks indicating if they're alternatives or all required
		SubtaskRelation m_subtasksRelation;

		///< Keeps track of all the semantic tasks for each dependency type
		DependencyMap	m_dependencyMap;
		
		///< If this SemanticTask is a simple task, this holds the corresponding motion task
		std::shared_ptr<MPTask>	m_simpleTask;

		///< Indicates if the assignment of the simple task is fixed
		bool m_fixedAssignment{false};

		//< Indicates if the task can be decomposed into subtasks
		bool m_decomposable{true};

		///@}
};

#endif
