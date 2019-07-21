#include "TaskPlan.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/HandoffAgent.h"

/**************************************Task Plan****************************/
TaskPlan::
TaskPlan(){}

TaskPlan::
TaskPlan(const TaskPlan& _other){
  m_coordinator = _other.m_coordinator;
  m_wholeTasks = _other.m_wholeTasks;
  m_subtaskMap = _other.m_subtaskMap;
  m_RAT = _other.m_RAT;
  m_TIM = _other.m_TIM;
  m_agentTaskMap = _other.m_agentTaskMap;
  m_taskDependencyMap = _other.m_taskDependencyMap;
  m_dummyMap = _other.m_dummyMap;
  m_memberAgents = _other.m_memberAgents;
  m_interactionTemplates = _other.m_interactionTemplates;
  m_taskCostMap = _other.m_taskCostMap;
}

TaskPlan::
~TaskPlan(){
  for(auto wholeTask : m_wholeTasks){
    delete wholeTask;
  }
}

TaskPlan&
TaskPlan::
operator=(const TaskPlan& _other){
  m_coordinator = _other.m_coordinator;
  m_wholeTasks = _other.m_wholeTasks;
  m_subtaskMap = _other.m_subtaskMap;
  m_RAT = _other.m_RAT;
  m_TIM = _other.m_TIM;
  m_agentTaskMap = _other.m_agentTaskMap;
  m_taskDependencyMap = _other.m_taskDependencyMap;
  m_dummyMap = _other.m_dummyMap;
  m_memberAgents = _other.m_memberAgents;
  m_interactionTemplates = _other.m_interactionTemplates;
  m_taskCostMap = _other.m_taskCostMap;
  return *this;
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
  InitializeCostMap();
  InitializeRAT();
  GenerateDummyAgents();
  InitializeAgentTaskMap();
}

/***********************************Accessors*************************/

void
TaskPlan::
InitializeAgentTaskMap(){
  for(auto agent : m_memberAgents){
    m_agentTaskMap[agent] = {};
  }
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

void
TaskPlan::
RemoveAllDependencies(){
  m_taskDependencyMap.clear();
}

double
TaskPlan::
GetPlanCost(WholeTask* _wholeTask){
  return m_taskCostMap[_wholeTask];
}

void
TaskPlan::
SetPlanCost(WholeTask* _wholeTask, double cost){
  m_taskCostMap[_wholeTask] = cost;
}

std::unordered_map<WholeTask*,double>&
TaskPlan::
GetTaskCostMap(){
  return m_taskCostMap;
}

double
TaskPlan::
GetEntireCost(){
  // TODO: make this adaptive!
  double max = 0;
  for (auto kv : m_taskCostMap){
    if (m_taskCostMap[kv.first] > max)
      max = m_taskCostMap[kv.first];
  }
  return max;
}

/*****************************************RAT Functions****************************************************/

void
TaskPlan::
InitializeRAT(){
  m_RAT.clear();
  for(auto agent : m_memberAgents){
    auto cfg = agent->GetRobot()->GetSimulationModel()->GetState();
    OccupiedInterval start(agent, cfg, cfg, 0.0, 0.0);
    m_RAT[agent].push_back(start);
  }
}

std::unordered_map<Agent*,std::list<OccupiedInterval>>&
TaskPlan::
GetRAT(){
  return m_RAT;
}

std::list<OccupiedInterval>
TaskPlan::
GetRobotAvailability(Agent* _agent){
  return m_RAT.at(_agent);
}

void
TaskPlan::
UpdateRAT(HandoffAgent* _agent, OccupiedInterval _interval){
  auto& avail = m_RAT[_agent];
  for(auto it = avail.begin(); it != avail.end(); it++){
    if(_interval < *it){
      avail.insert(it,_interval);
      return;
    }
  }
  avail.push_back(_interval);
}

void
TaskPlan::
ClearRobotAvailability(HandoffAgent* _agent){
  m_RAT[_agent].clear();
}
/*****************************************TIM Functions****************************************************/

std::unordered_map<WholeTask*,std::list<OccupiedInterval>>&
TaskPlan::
GetTIM() {
  return m_TIM;
}

std::list<OccupiedInterval>
TaskPlan::
GetTaskIntervals(WholeTask* _wholeTask){
  return m_TIM[_wholeTask];
}

void
TaskPlan::
UpdateTIM(WholeTask* _wholeTask, OccupiedInterval _interval){
  auto& plan = m_TIM[_wholeTask];
  if(plan.empty()){
    plan.push_back(_interval);
    return;
  }
  if(*plan.end() < _interval){
    plan.push_back(_interval);
    return;
  }
  for(auto it = plan.begin(); it != plan.end(); it++){
    if(_interval < *it){
      plan.insert(it, _interval);
      return;
    }
  }
}

void
TaskPlan::
ClearTaskIntervals(WholeTask* _wholeTask){
  m_TIM[_wholeTask].clear();
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

std::unordered_map<std::shared_ptr<MPTask>, WholeTask*>&
TaskPlan::
GetSubtaskMap(){
  return m_subtaskMap;
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

/***********************************Helpers*******************************/

void
TaskPlan::
InitializeCostMap(){
  for(auto task : m_wholeTasks){
    m_taskCostMap[task] = MAX_DBL;
  }
}
