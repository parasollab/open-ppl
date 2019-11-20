#include "Decomposition.h"

/*---------------------------- Construction ---------------------------*/

Decomposition::
Decomposition(std::shared_ptr<SemanticTask> _mainTask) {
	m_mainTask = _mainTask;
	m_simpleTasks = m_mainTask->SetDependencies();
}
	
Decomposition::
Decomposition(XMLNode& _node) {
	
	std::unordered_map<std::string,std::shared_ptr<SemanticTask>> labelMap;
	
	std::string mainTask = _node.Read("taskLabel", true, "", 
							"Label for the highest level semantic task.");

	// create all the semantic tasks
	for(auto child : _node) {
		auto task = std::shared_ptr<SemanticTask>(new SemanticTask(child);
		labelMap[task->GetLabel()] = task;
	}

	// set the highest level task
	m_mainTask = labelMap[mainTask];

	// build the parent connections between the semantic tasks
	for(auto child : _node) {
		std::string label = child.Read("label", true, "", "Label for semantic task.");
		auto task = labelMap[label];

		for(auto grandchild : child) {
			if(grandchild.Name() == "Subtask") {
				std::string subtaskLabel = grandchild.Read("label", true, "",
													"Label for semantic task.");
				auto subtask = labelMap[subtaskLabel];
				subtask->SetParent(task);
			}
			else if(grandchild.Name() == "Dependency") {
				std::string subtaskLabel = grandchild.Read("label", true, "",
													"Label for semantic task.");
				auto subtask = labelMap[subtaskLabel];

				std::string dependencyType = grandchild.Read("type", true, "",
													"Type of dependency");

				SemanticTask::DependencyType type;

				if(dependencyType == "Initiation")
					type = SemanticTask::DependencyType::Initiation;
				else if(dependencyType == "Completion")
					type = SemanticTask::DependencyType::Completion;
				else if(dependencyType == "Asynchronous")
					type = SemanticTask::DependencyType::Asynchronous;
				else if(dependencyType == "Synchronous")
					type = SemanticTask::DependencyType::Synchronous;

				subtask->AddDependency(subtask,type);
			}
		}
	}

	// set the dependencies between the simple tasks
	m_simpleTasks = m_mainTask->SetDependencies();
}

Decomposition::
~Decomposition() {}

	
/*---------------------------- Accessors ---------------------------*/

std::shared_ptr<SemanticTask> 
Decomposition::
GetMainTask() {
	return m_mainTask;
}

std::vector<std::shared_ptr<SemanticTask>>& 
Decomposition::
GetSimpleTasks() {
	return m_simpleTasks;
}

