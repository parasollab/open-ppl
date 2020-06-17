#ifndef SEMANTIC_TASK_H_
#define SEMANTIC_TASK_H_

#include "MPProblem/MPTask.h"

#include <unordered_map>
#include <unordered_set>

class Decomposition;
class MPProblem;

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

		typedef std::unordered_map<DependencyType, std::unordered_set<SemanticTask*>, 
															 std::hash<int>> DependencyMap;

		///@}
		///@name Construction
		///@{

		SemanticTask();

		SemanticTask(MPProblem* _problem, XMLNode& _node, Decomposition* _decomp);

		SemanticTask(std::string _label, SemanticTask* _parent, Decomposition* _decomp,
								 SubtaskRelation _relation, bool _decomposable, bool _fixed, 
								 std::shared_ptr<MPTask> _simpleTask = nullptr);

		SemanticTask(SemanticTask* _parent, Decomposition* _decomp, std::shared_ptr<MPTask> _simpleTask,
								 bool _decomposable=true);

		~SemanticTask();

		///@}
		///@name Accessors
		///@{

		std::string GetLabel() const;

		///< Sets the dependencies of all the semantic tasks below this in the hierarchy
		std::vector<SemanticTask*> SetDependencies();

		///< Indicates if this is a simple task or not
		bool IsDecomposable();

		void SetMotionTask(std::shared_ptr<MPTask> _motion);

		std::shared_ptr<MPTask> GetMotionTask();

		SemanticTask* GetParent();

		void SetParent(SemanticTask* _parent);

		std::unordered_set<SemanticTask*> AddDependency(SemanticTask* _task, DependencyType _type);

		DependencyMap& GetDependencies();

		bool IsFixedAssignment();

		void AddSubtask(SemanticTask* _task);

		std::vector<SemanticTask*> GetSubtasks();

		SubtaskRelation GetSubtaskRelation();

		//void SetCompletionFunction(TBDFunction _func);

		//TBDFunction GetCompletionFunction();

		///@}

	private:
		///@name Helper Functions
		///@{

		void ParseDependency(MPProblem* _problem, XMLNode& _node, Decomposition* _decomp);

		///@}
		///@name Internal State
		///@{

		///< Label distinguishing this semantic task
		std::string m_label;

		///< Parent SemanticTask that includes this task in its decomposition
		SemanticTask* m_parent{nullptr};

		///< If this SemanticTask is a simple task, this holds the corresponding motion task
		std::shared_ptr<MPTask>	m_simpleTask;

		///< Indicates if the assignment of the simple task is fixed
		bool m_fixedAssignment{false};

		///< Indicates if the task can be decomposed into subtasks
		bool m_decomposable{true};

		///< Set of subtasks that make up this task's decomposition
		std::vector<SemanticTask*> m_subtasks;

		///< Relationship between the subtasks indicating if they're alternatives or all required
		SubtaskRelation m_subtasksRelation;

		///< Keeps track of all the semantic tasks for each dependency type
		DependencyMap	m_dependencyMap;
		

		//TBDFunction m_completionFunction;

		///@}
};

#endif
