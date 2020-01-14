#include "SemanticTask.h"

#include "Decomposition.h"
#include "MPProblem/MPProblem.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

/*---------------------------- Construction -----------------------*/

SemanticTask::
SemanticTask() {}

SemanticTask::
SemanticTask(MPProblem* _problem, XMLNode& _node, Decomposition* _decomp) {

	m_label = _node.Read("label", true, "", "Unique label to identify this task.");

	std::string relation = _node.Read("subtaskRelation",false,"",
										"Indicates if this task's subtasks are alternatives or both required.");

	std::transform(relation.begin(), relation.end(), relation.begin(), ::tolower);

	if(relation == "and")
		m_subtasksRelation = SubtaskRelation::AND;
	else if(relation == "xor" or relation == "or")
		m_subtasksRelation = SubtaskRelation::XOR;
	

	m_decomposable = _node.Read("decomposable", false, true, 
				"Indicates if a task is decomposable into subtasks.");

	m_fixedAssignment = _node.Read("fixedAllocation", false, false, 
				"Indicates if the simpe tasks within this have fixed allocations.");

	std::string parentLabel = _node.Read("parent", false, "", "Label for task's parent.");
	auto parent = _decomp->GetTask(parentLabel);
	if(parent) {
		this->SetParent(parent);
		parent->AddSubtask(this);
	}

	for(auto& child : _node) {
		if(child.Name() == "Task") {
			m_simpleTask = std::shared_ptr<MPTask>(new MPTask(_problem,child));
		}
		else if(child.Name() == "Dependency") {
			ParseDependency(_problem, _node, _decomp);
		}	
	}

	//m_subtasks
	//m_dependencyMap

}

SemanticTask::
SemanticTask(SemanticTask* _parent, std::shared_ptr<MPTask> _simpleTask,
						 bool _decomposable) : m_parent(_parent), m_simpleTask(_simpleTask), m_decomposable(_decomposable) {}


SemanticTask::
~SemanticTask() {}
	
void
SemanticTask::
ParseDependency(MPProblem* _problem, XMLNode& _node, Decomposition* _decomp) {

	for(auto& child : _node) {
		if(child.Name() == "Dependency") {
			std::string depTaskLabel = child.Read("task", true, "",
													"Label for semantic task.");
			auto depTask = _decomp->GetTask(depTaskLabel);

			std::string dependencyType = child.Read("type", true, "",
													"Type of dependency");
			std::transform(dependencyType.begin(), dependencyType.end(), dependencyType.begin(), ::tolower);

			SemanticTask::DependencyType type = SemanticTask::DependencyType::None;

			if(dependencyType == "initiation")
				type = SemanticTask::DependencyType::Initiation;
			else if(dependencyType == "completion")
				type = SemanticTask::DependencyType::Completion;
			else if(dependencyType == "asynchronous")
				type = SemanticTask::DependencyType::Asynchronous;
			else if(dependencyType == "synchronous")
				type = SemanticTask::DependencyType::Synchronous;
			else 
				throw RunTimeException(WHERE, "Unknown dependency type: " + dependencyType);
			this->AddDependency(depTask,type);
		}
		//else if(child.Name() == "Task") {
		//	std::shared_ptr<MPTask> simpleTask = std::shared_ptr<MPTask>(new MPTask(_problem,grandchild));
		//	task->SetMotionTask(simpleTask);
		//	AddSimpleTask(task.get());
		//}
	}
}
/*---------------------------- Accessors -----------------------*/
	
std::string 
SemanticTask::
GetLabel() const {
	return m_label;
}
std::vector<SemanticTask*> 
SemanticTask::
SetDependencies() {

	std::unordered_map<SemanticTask*,
							std::vector<SemanticTask*>> simpleTaskDecompositions;

	std::vector<SemanticTask*> simpleTasks;

	// Check if this is a simple task and if so add it to the set of simple tasks for itself
	if(m_simpleTask) {
		return {this};
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
				if(depTask->GetParent() != this) {
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

void
SemanticTask::
SetMotionTask(std::shared_ptr<MPTask> _motion) {
	m_simpleTask = _motion;
}

std::shared_ptr<MPTask> 
SemanticTask::
GetMotionTask() {
	return m_simpleTask;
}

SemanticTask*
SemanticTask::
GetParent() {
	return m_parent;
}

void
SemanticTask::
SetParent(SemanticTask* _parent) {
	m_parent = _parent;
}

void 
SemanticTask::
AddDependency(SemanticTask* _task, DependencyType _type) {
	m_dependencyMap[_type].push_back(_task);
}

std::unordered_map<SemanticTask::DependencyType,std::vector<SemanticTask*>,std::hash<int>>&
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
AddSubtask(SemanticTask* _task) {
	m_subtasks.push_back(_task);
}

std::vector<SemanticTask*> 
SemanticTask::
GetSubtasks() {
	return m_subtasks;
}
		
SemanticTask::SubtaskRelation 
SemanticTask::
GetSubtaskRelation() {
	return m_subtasksRelation;
}

/*		
void 
SemanticTask::
SetCompletionFunction(TBDFunction _func) {
	m_completionFunction = _func;
}

TBDFunction 
SemanticTask::
GetCompletionFunction() {
	return m_completionFunction;
}
*/
