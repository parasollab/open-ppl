#include "Decomposition.h"

#include "MPProblem/MPProblem.h"
#include "Utilities/XMLNode.h"

/*---------------------------- Construction ---------------------------*/

Decomposition::
Decomposition(std::shared_ptr<SemanticTask> _mainTask) {
	m_mainTask = _mainTask.get();
	//m_simpleTasks = m_mainTask->SetDependencies();
}
	
Decomposition::
Decomposition(XMLNode& _node, MPProblem* _problem) {
	
	std::string mainTask = _node.Read("taskLabel", true, "", 
							"Label for the highest level semantic task.");

	m_label = _node.Read("label", true, "", 
							"Unique label for the decomposition");

	std::string coordinator = _node.Read("coordinator", true, "",
							"Indicates whcih robot is responsible for coordinating this decompostion.");

	m_coordinator = _problem->GetRobot(coordinator);

	// create all the semantic tasks
	for(auto& child : _node) {
		if(child.Name() == "SemanticTask") {
			auto task = std::shared_ptr<SemanticTask>(new SemanticTask(_problem,child,this));
			if(m_taskMap[task->GetLabel()])
				throw RunTimeException(WHERE, "SemanticTask labels must be unique within a decomposition.");
			m_taskMap[task->GetLabel()] = task;
		}
	}

	// set the highest level task
	m_mainTask = m_taskMap[mainTask].get();

	// set the dependencies between the simple tasks
	//m_simpleTasks = m_mainTask->SetDependencies();
}

Decomposition::
~Decomposition() {}

	
/*---------------------------- Accessors ---------------------------*/
const std::string
Decomposition::
GetLabel() const {
	return m_label;
}

Robot*
Decomposition::
GetCoordinator() const {
	return m_coordinator;
}

SemanticTask*
Decomposition::
GetMainTask() {
	return m_mainTask;
}

void 
Decomposition::
AddTask(std::shared_ptr<SemanticTask> _task) {
	m_taskMap[_task->GetLabel()] = _task;
}

SemanticTask* 
Decomposition::
GetTask(std::string _label) {
	//TODO::Add check if task exists
	return m_taskMap[_label].get();
}



std::vector<SemanticTask*>& 
Decomposition::
GetSimpleTasks() {
	return m_simpleTasks;
}

std::vector<SemanticTask*>& 
Decomposition::
GetMotionTasks() {
	return m_motionTasks;
}

void 
Decomposition::
AddSimpleTask(SemanticTask* _task) {
	m_simpleTasks.push_back(_task);
}

void 
Decomposition::
AddMotionTask(SemanticTask* _task) {
	m_motionTasks.push_back(_task);
}

/*--------------------------------- Helper Functions ------------------*/

void
Decomposition::
ParseTask(XMLNode& _node, MPProblem* _problem) {
	std::string label = _node.Read("label", true, "", "Label for semantic task.");
	auto task = m_taskMap[label];

	std::string parentLabel = _node.Read("parent", false, "", "Label for task's parent.");
	auto parent = m_taskMap[parentLabel];
	if(parent) {
		task->SetParent(parent.get());
		parent->AddSubtask(task.get());
	}

	for(auto child : _node) {
		if(child.Name() == "Dependency") {
			std::string depTaskLabel = child.Read("task", true, "",
													"Label for semantic task.");
			auto depTask = m_taskMap[depTaskLabel];

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
			task->AddDependency(depTask.get(),type);
		}
		//else if(child.Name() == "Task") {
		//	std::shared_ptr<MPTask> simpleTask = std::shared_ptr<MPTask>(new MPTask(_problem,grandchild));
		//	task->SetMotionTask(simpleTask);
		//	AddSimpleTask(task.get());
		//}
	}
}
