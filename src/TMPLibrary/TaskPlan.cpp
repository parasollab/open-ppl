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

/***********************************WholeTask Interactions*******************************/

std::vector<WholeTask*>&
TaskPlan::
GetWholeTasks(){
	return m_wholeTasks;
}

void
TMPStrategyMethod::
CreateWholeTasks(std::vector<std::shared_ptr<MPTask>> _tasks){
  for(auto task : _tasks){
    WholeTask* wholeTask = new WholeTask();
    wholeTask->m_task = task;
    for(auto const& elem : m_dummyMap){
      wholeTask->m_startPoints[elem.first] = {};
      wholeTask->m_goalPoints[elem.first] = {};
      wholeTask->m_startVIDs[elem.first] = {};
      wholeTask->m_goalVIDs[elem.first] = {};
    }
    m_wholeTasks.push_back(wholeTask);
  }
}

WholeTask*
TMPStrategyMethod::
GetWholeTask(std::shared_ptr<MPTask> _subtask){
	return m_subtaskMap[_subtask];
}

std::shared_ptr<MPTask> 
TaskPlan::
GetNextSubtask(WholeTask* _wholeTask){
  if(_wholeTask->m_subtaskIterator == _wholeTask->m_subtasks.size())
    return nullptr;
  return _wholeTask->m_subtasks[_wholeTask->m_subtaskIterator++];
}

void 
TaskPlan::
AddSubtask(std::shared_ptr<MPTask> _subtask){
  if(m_unassignedTasks.empty()){
    m_unassignedTasks.push_back(_subtask);
    return;
  }
  for(auto it = m_unassignedTasks.begin(); it != m_unassignedTasks.end(); it++){
    if(_subtask->GetEstimatedStartTime() < it->get()->GetEstimatedStartTime()){
      m_unassignedTasks.insert(it, _subtask);
      return;
    }
  }
  m_unassignedTasks.push_back(_subtask);
}

Agent*
TaskPlan::
GetLastAgent(WholeTask* _wholeTask){
  if(_wholeTask->m_subtaskIterator == 0)
    return nullptr;
  auto lastSubtask = _wholeTask->m_subtasks[_wholeTask->m_subtaskIterator - 1];
  return static_cast<HandoffAgent*>(lastSubtask->GetRobot()->GetAgent());
}
