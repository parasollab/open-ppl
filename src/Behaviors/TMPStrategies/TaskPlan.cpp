#include "TaskPlan.h"

std::list<std::shared_ptr<MPTask>>
TaskPlan::
GetAgentTasks(Agent* _agent){
  return m_agentTaskMap[_agent];
}

std::list<std::shared_ptr<MPTask>>
TaskPlan::
GetTaskDependencies(std::shared_ptr<MPTask> _task){
  return m_taskDependencyMap[_task];
}

void
TaskPlan::
AddSubtask(Agent* _agent, std::shared_ptr<MPTask> _task){
	auto& tasks = m_agentTaskMap[_agent];
	tasks.push_back(_task);
}

void 
TaskPlan::
AddDependency(std::shared_ptr<MPTask> _first, std::shared_ptr<MPTask> _second){
	auto& dependencies = m_taskDependencyMap[_second];
	dependencies.push_back(_first);
}

void 
TaskPlan::
RemoveLastDependency(std::shared_ptr<MPTask> _task){
	auto& dependencies = m_taskDependencyMap[_task];
	dependencies.pop_back();
}

