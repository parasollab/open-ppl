#include "SemanticTask.h"

#include "Utilities/PMPLExceptions.h"

/*---------------------------- Construction -----------------------*/

SemanticTask::
SemanticTask() {}

SemanticTask::
SemanticTask(XMLNode& _node) {

	//m_decomposable
	//m_fixedAllocation
	//m_label
	//m_simpleTask
	//m_subtasks
	//m_dependencyMap

}

SemanticTask::
SemanticTask(std::shared_ptr<MPTask> _simpleTask, std::shared_ptr<SemanticTask> _parent, 
						 bool _decomposable) {
	m_simpleTask = _simpleTask;
	m_parent = _parent;
	m_decomposable = _decomposable;
}


SemanticTask::
~SemanticTask() {}
	
/*---------------------------- Accessors -----------------------*/
	
std::string 
SemanticTask::
GetLabel() const {
	return m_label;
}
std::vector<std::shared_ptr<SemanticTask>> 
SemanticTask::
SetDependencies() {

	std::unordered_map<std::shared_ptr<SemanticTask>,
							std::vector<std::shared_ptr<SemanticTask>>> simpleTaskDecompositions;

	std::vector<std::shared_ptr<SemanticTask>> simpleTasks;

	// Check if this is a simple task and if so add it to the set of simple tasks for itself
	if(m_simpleTask) {
		return {std::shared_ptr<SemanticTask>(this)};
	}

	// Collect all of the simple tasks composing the subtasks
	for(auto subtask : m_subtasks) {
		simpleTaskDecompositions[subtask] = subtask->SetDependencies();
		for(auto simple : simpleTaskDecompositions[subtask]){
			simpleTasks.push_back(simple);
		}
	}

	for(auto taskDecomp : simpleTaskDecompositions) {
		auto task = taskDecomp.first; 
		for(auto dependencyTasks : task->GetDependencies()) {

			// iterate through dependent semantic tasks
			// make sure they all have THIS task as their parent
			// add dependencies from their set of simple tasks

			for(auto depTask : dependencyTasks.second) {
				if(depTask->GetParent().get() != this) {
					throw RunTimeException(WHERE,
						"Decomposition input has a listed dependency between non-sibling tasks.");
				}
				
				for(auto simple1 : simpleTaskDecompositions[task]) {
					for(auto simple2 : simpleTaskDecompositions[depTask]) {
						simple1->AddDependency(simple2,dependencyTasks.first);
					}
				}
			}
		}
	}

	return simpleTasks;
}

bool 
SemanticTask::
IsDecomposable() {
	return m_decomposable;
}

std::shared_ptr<MPTask> 
SemanticTask::
GetMotionTask() {
	return m_simpleTask;
}

std::shared_ptr<SemanticTask>
SemanticTask::
GetParent() {
	return m_parent;
}

void
SemanticTask::
SetParent(std::shared_ptr<SemanticTask> _parent) {
	m_parent = _parent;
}

void 
SemanticTask::
AddDependency(std::shared_ptr<SemanticTask> _task, DependencyType _type) {
	m_dependencyMap[_type].push_back(_task);
}

std::unordered_map<SemanticTask::DependencyType,std::vector<std::shared_ptr<SemanticTask>>,std::hash<int>>&
SemanticTask::
GetDependencies() {
	return m_dependencyMap;
}

bool 
SemanticTask::
IsFixedAssignment() {
	return m_fixedAssignment;
}

void 
SemanticTask::
AddSubtask(std::shared_ptr<SemanticTask> _task) {
	m_subtasks.push_back(_task);
}
