#include "TaskPlan.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/HandoffAgent.h"


TaskPlan::
~TaskPlan(){
	for(auto wholeTask : m_wholeTasks){
		delete wholeTask;
	}
}

void
TaskPlan::
Initialize(){
	if(!m_coordinator){
		throw RunTimeException(WHERE, "TaskPlan has no coordinator.");
	}
	else if(m_memberAgents.empty()){
		throw RunTimeException(WHERE, "TaskPlan has no member agents.");
	}
	else if(m_wholeTasks.empty()){
		throw RunTimeException(WHERE, "TaskPlan has no tasks to plan for.");
	}	
	InitializeRAT();
	GenerateDummyAgents();
}

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
AddSubtask(HandoffAgent* _agent, std::shared_ptr<MPTask> _task){
	auto& tasks = m_agentTaskMap[_agent];
	tasks.push_back(_task);
}

void
TaskPlan::
AddSubtask(HandoffAgent* _agent, std::shared_ptr<MPTask> _task, WholeTask* _wholeTask){
	auto& tasks = m_agentTaskMap[_agent];
	tasks.push_back(_task);
	AddSubtaskToWholeTask(_task,_wholeTask);
	_wholeTask->m_agentAssignment.push_back(_agent);
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

/*****************************************RAT Functions****************************************************/

void
TaskPlan::
InitializeRAT(){
	for(auto agent : m_memberAgents){
		auto cfg = agent->GetRobot()->GetSimulationModel()->GetState();
		m_RAT[agent->GetRobot()->GetLabel()] = std::pair<Cfg,double>(cfg, 0.0);
	}
}

std::unordered_map<std::string,std::pair<Cfg,double>>& 
TaskPlan::
GetRAT(){
	return m_RAT;
}

std::pair<Cfg,double>
TaskPlan::
GetRobotAvailability(HandoffAgent* _agent){
	return m_RAT.at(_agent->GetRobot()->GetLabel());
}

void
TaskPlan::
UpdateRAT(HandoffAgent* _agent, std::pair<Cfg,double> _avail){
	m_RAT[_agent->GetRobot()->GetLabel()] = _avail;
}
/***********************************WholeTask Interactions*******************************/

void
TaskPlan::
AddWholeTask(WholeTask* _wholeTask){
	m_wholeTasks.push_back(_wholeTask);
}

std::vector<WholeTask*>&
TaskPlan::
GetWholeTasks(){
	return m_wholeTasks;
}

void
TaskPlan::
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

void
TaskPlan::
SetWholeTaskOwner(std::shared_ptr<MPTask> _subtask, WholeTask* _wholeTask){
	m_subtaskMap[_subtask] = _wholeTask;
}

WholeTask*
TaskPlan::
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

HandoffAgent*
TaskPlan::
GetLastAgent(WholeTask* _wholeTask){
  if(_wholeTask->m_subtaskIterator == 0)
    return nullptr;
  auto lastSubtask = _wholeTask->m_subtasks[_wholeTask->m_subtaskIterator - 1];
  return static_cast<HandoffAgent*>(lastSubtask->GetRobot()->GetAgent());
}

void
TaskPlan::
AddSubtaskToWholeTask(std::shared_ptr<MPTask> _subtask, WholeTask* _wholeTask){
	_wholeTask->m_subtasks.push_back(_subtask);
	SetWholeTaskOwner(_subtask, _wholeTask);
}

/***********************************Agent Accessors*******************************/

void
TaskPlan::
SetCoordinator(Coordinator* _c){
	m_coordinator = _c;
}

Coordinator*
TaskPlan::
GetCoordinator(){
	return m_coordinator;
}

void
TaskPlan::
LoadTeam(std::vector<HandoffAgent*> _team){
	m_memberAgents = _team;
}

std::vector<HandoffAgent*>& 
TaskPlan::
GetTeam(){
	return m_memberAgents;
}

void
TaskPlan::
GenerateDummyAgents(){
  m_dummyMap.clear();
  // Load the dummyMap, which stores a dummy agent for each agent capability.
  for(auto agent : m_memberAgents){
    std::string capability = agent->GetCapability();
    if(m_dummyMap.find(capability) == m_dummyMap.end()){
      m_dummyMap[capability] = agent;
    }
  }
}

HandoffAgent*
TaskPlan::
GetCapabilityAgent(std::string _robotType){
	return m_dummyMap[_robotType];
}

std::unordered_map<std::string,HandoffAgent*>&
TaskPlan::
GetDummyMap(){
	return m_dummyMap;
}

/***********************************IT Accessors*******************************/

std::vector<std::shared_ptr<InteractionTemplate>>&
TaskPlan::
GetInteractionTemplates(){
  return m_interactionTemplates;
}

void
TaskPlan::
AddInteractionTemplate(InteractionTemplate* _it){
  m_interactionTemplates.emplace_back(std::unique_ptr<InteractionTemplate>(_it));
}

