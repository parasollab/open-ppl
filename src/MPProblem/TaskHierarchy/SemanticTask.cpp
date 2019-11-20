#include "SemanticTask.h"

/*---------------------------- Construction -----------------------*/

SemanticTask::
SemanticTask() {}

SemanticTask::
SemanticTask(XMLNode& _node) {

}

SemanticTask::
~SemanticTask() {}
	
/*---------------------------- Accessors -----------------------*/
	
std::vector<std::shared_ptr<SemanticTask>> 
SemanticTask::
SetDependencies() {

	std::unordered_map<SemanticTask,std::vector<SemanticTask>> simpleTaskDecompositions;
	std::vector<std::shared_ptr<SemanticTask>> simpleTasks;

	// Check if this is a simple task and if so add it to the set of simple tasks for itself
	if(m_simpleTask) {
		return {this};
	}

	// Collect all of the simple tasks composing the subtasks
	for(auto subtask : m_subtasks) {
		simpleTaskDecompositions[subtask] = subtask->SetDependencies();
		simpleTasks += simpleTaskDecompositions[subtask];
	}

	for(auto taskDecomp : simpleTaskDecompositions) {
		auto task = taskDecomp.first; 
		for(auto dependencyTasks : task->GetDependencies()) {

			// iterate through dependent semantic tasks
			// make sure they all have THIS task as their parent
			// add dependencies from their set of simple tasks

			for(auto depTask : dependencyTasks.second) {
				if(dependentTask->GetParent() != this) {
					throw RunTimeException(WHERE,
						"Decomposition input has a listed dependency between non-sibling tasks.");
				}
				
				for(auto simple1 : simpleTaskDecompositions[task]) {
					for(auto simple2 : simpleTaskDecomposition[depTask]) {
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
	return !subtask.empty();
}

std::shared_ptr<MPTask> 
SemanticTask::
GetMotionTask() {
	return m_simpleTask;
}

std::shared_ptr<SemanticTask>
SemanticTask::
GetParent() {
	reutrn m_parent;
}

bool
SemanticTask::
IsFixedAssignment() {
	return m_fixedAssignment;
}
		
void 
SemanticTask::
AddDependency(std::shared_ptr<SemanticTask> _task, DependencyType _type) {
	m_dependencyMap[_type].push_back(_task);
}
