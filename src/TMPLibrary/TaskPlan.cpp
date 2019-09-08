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
	m_positiveConstraints = _other.m_positiveConstraints;
	m_posInstantConstraints = _other.m_posInstantConstraints;
}

TaskPlan::
~TaskPlan(){
	//TODO::turn these into shared_ptr
  /*for(auto wholeTask : m_wholeTasks){
    delete wholeTask;
  }*/
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
	m_positiveConstraints = _other.m_positiveConstraints;
	m_posInstantConstraints = _other.m_posInstantConstraints;
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
	InitializePositiveConstraints();
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

std::pair<double,double>
TaskPlan::
GetPlanCost(WholeTask* _wholeTask){
  return m_taskCostMap[_wholeTask];
}

void
TaskPlan::
SetPlanCost(WholeTask* _wholeTask, double _start, double _end){
  m_taskCostMap[_wholeTask] = std::make_pair(_start,_end);
}

std::unordered_map<WholeTask*,std::pair<double,double>>&
TaskPlan::
GetTaskCostMap(){
  return m_taskCostMap;
}

double
TaskPlan::
GetEntireCost(bool _makespan){
  // TODO: make this adaptive!
  // max = makespan
  // sum = SOC (sum of costs)
  double max = 0;
  double sum = 0;
  for (auto kv : m_taskCostMap){
    sum += kv.second.second;// - kv.second.first;
    if (kv.second.second > max)
      max = kv.second.second;
  }
	if(_makespan)
		return max;
  return sum;
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
  if(plan.back() < _interval){
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
  for(auto& wholeTask : m_wholeTasks){
    wholeTask->m_agentAssignment.clear();
  }
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
    m_taskCostMap[task] = std::make_pair(0,MAX_DBL);
  }
}

/***********************************Positive Constraints*********************/

void 
TaskPlan::
InitializePositiveConstraints() {
	for(auto task : m_wholeTasks) {
		m_positiveConstraints[task] = {};
		m_posInstantConstraints[task] = {};
	}
}

void 
TaskPlan::
AddPositiveConstraint(WholeTask* _task, OccupiedInterval _constraint) {
	auto& constraints = m_positiveConstraints[_task];
  if(constraints.empty()){
    constraints.push_back(_constraint);
    return;
  }
  if(constraints.back() < _constraint){
    constraints.push_back(_constraint);
    return;
  }
  for(auto it = constraints.begin(); it != constraints.end(); it++){
    if(_constraint < *it){
      constraints.insert(it, _constraint);
      return;
    }
  }
}

std::list<OccupiedInterval>&
TaskPlan::
GetPositiveTaskConstraints(WholeTask* _task) {
	return m_positiveConstraints[_task];
}

std::unordered_map<WholeTask*,std::list<OccupiedInterval>>&
TaskPlan::
GetPositiveConstraints() {
	return m_positiveConstraints;
}

void 
TaskPlan::
AddPositiveInstantConstraint(WholeTask* _task, HandoffAgent* _agent, double _instant) {
	auto pair = std::make_pair(_agent,_instant);
	auto& instants = m_posInstantConstraints[_task];
  if(instants.empty()){
    instants.push_back(pair);
    return;
  }
  if(instants.back().second < _instant){
    instants.push_back(pair);
    return;
  }
  for(auto it = instants.begin(); it != instants.end(); it++){
    if(_instant < (*it).second){
      instants.insert(it, pair);
      return;
    }
  }
}

std::list<std::pair<HandoffAgent*,double>>&
TaskPlan::
GetPositiveInstantTaskConstraints(WholeTask* _task) {
	return m_posInstantConstraints[_task];
}

std::unordered_map<WholeTask*,std::list<std::pair<HandoffAgent*,double>>>&
TaskPlan::
GetPositiveInstantConstraints() {
	return m_posInstantConstraints;
}
/**************************************Discrete Stuff****************************/
void 
TaskPlan::
AddAgentAllocation(HandoffAgent* _agent, DiscreteAgentAllocation _allocation) {
/*	auto& allocations = m_agentAllocationMap[_agent];
	if(allocations.empty() or allocations.back().m_startTime < _allocation.m_startTime) {
		allocations.push_back(_allocation);
		return;
	}

	for(auto iter = allocations.begin(); iter != allocations.end(); iter++) {
		if(iter->m_startTime > _allocation.m_startTime) {
			allocations.insert(iter,_allocation);
			return;
		}
	}*/
	m_agentAllocationMap[_agent].push_back(_allocation);
}

std::unordered_map<HandoffAgent*,std::vector<DiscreteAgentAllocation>>&
TaskPlan::
GetAllAgentAllocations() {
	return m_agentAllocationMap;
}

std::vector<DiscreteAgentAllocation>
TaskPlan::
GetAgentAllocations(HandoffAgent* _agent) {
	return m_agentAllocationMap[_agent];
}

void
TaskPlan::
SetAgentAllocations(AgentAllocationMap _allocs) {
	m_agentAllocationMap = _allocs;
}
