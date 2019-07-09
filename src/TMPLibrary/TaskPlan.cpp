#include "TaskPlan.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/HandoffAgent.h"

OccupiedInterval::
OccupiedInterval(Robot* _r, Cfg _sL, Cfg _eL, double _sT, double _eT) : m_robot(_r),
m_startLocation(_sL), m_endLocation(_eL), m_startTime(_sT), m_endTime(_eT){ }

Robot*
OccupiedInterval::
GetRobot(){
  return m_robot;
}

Cfg
OccupiedInterval::
GetStartLocation(){
  return m_startLocation;
}

Cfg
OccupiedInterval::
GetEndLocation(){
  return m_endLocation;
}

double
OccupiedInterval::
GetStartTime(){
  return m_startTime;
}

double
OccupiedInterval::
GetEndTime(){
  return m_endTime;
}

std::pair<Cfg, double>
OccupiedInterval::
GetStart(){
  return std::make_pair(m_startLocation, m_startTime);
}

std::pair<Cfg, double>
OccupiedInterval::
GetEnd(){
  return std::make_pair(m_endLocation, m_endTime);
}

void
OccupiedInterval::
SetStartLocation (Cfg _start)
{
  m_startLocation = _start;
}

void
OccupiedInterval::
SetEndLocation (Cfg _end)
{
  m_endLocation = _end;
}
void
OccupiedInterval::
SetStartTime (double _start)
{
  m_startTime = _start;
}
void
OccupiedInterval::
SetEndTime (double _end)
{
  m_endTime = _end;
}
void
OccupiedInterval::
SetStart(Cfg _startLoc, double _startTime)
{
  m_startLocation = _startLoc;
  m_startTime = _startTime;
}

void
OccupiedInterval::
SetEnd(Cfg _endLoc, double _endTime)
{
  m_endLocation = _endLoc;
  m_endTime = _endTime;
}

bool
OccupiedInterval::
CheckTimeOverlap(OccupiedInterval _interval){
  if (m_robot != _interval.GetRobot()){
    if (_interval.GetStartTime() >= m_startTime || _interval.GetStartTime() < m_endTime)
      return true;
    else if (_interval.GetEndTime() > m_startTime || _interval.GetEndTime() <= m_endTime)
      return true;
  }
  else{
    if (_interval.GetStartTime() >= m_startTime || _interval.GetStartTime() >= m_endTime)
      return true;
    else if (_interval.GetEndTime() >= m_startTime || _interval.GetEndTime() >= m_endTime)
      return true;
  }
  return false;
}

void
OccupiedInterval::
MergeIntervals(std::list <OccupiedInterval>& _intervals){
  std::list<OccupiedInterval>::iterator it = _intervals.begin();
  while (it != _intervals.end()){
    auto next = std::next(it);
    double nextTime = next -> GetStartTime();
    if (it -> GetRobot() != next -> GetRobot()){
      throw std::runtime_error("Merge Intervals - Different robots exist within set of occupied intervals");
    }

    if (nextTime >= it->GetStartTime() && nextTime <= it -> GetEndTime()){
      if(next->GetEndTime() > it->GetEndTime()){
        it -> SetEndTime(next -> GetEndTime());
        _intervals.erase(next);
      }
      else
        _intervals.erase(next);
    }
    else
      it ++;
  }
}

bool
OccupiedInterval::
operator<(OccupiedInterval _interval){
  return m_startTime < _interval.GetStartTime();
}

std::string
OccupiedInterval::
Print(){
	std::string output = "Start time: " + std::to_string(m_startTime) +
											 " Start Location: " + m_startLocation.PrettyPrint() +
											 "\nEnd Time: " + std::to_string(m_endTime) + 
											 " End Location: " + m_endLocation.PrettyPrint();
	return output;
}

/**************************************Task Plan****************************/

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
		OccupiedInterval* start = new OccupiedInterval(agent->GetRobot(), cfg, cfg, 0.0, 0.0);
		m_RAT[agent->GetRobot()->GetLabel()].push_back(start);
	}
}

std::unordered_map<std::string,std::list<OccupiedInterval*>>&
TaskPlan::
GetRAT(){
	return m_RAT;
}

std::list<OccupiedInterval*>
TaskPlan::
GetRobotAvailability(HandoffAgent* _agent){
	return m_RAT.at(_agent->GetRobot()->GetLabel());
}

void
TaskPlan::
UpdateRAT(HandoffAgent* _agent, OccupiedInterval* _interval){
	auto& avail = m_RAT[_agent->GetRobot()->GetLabel()];
	for(auto it = avail.begin(); it != avail.end(); it++){
		if(_interval < *it){
			avail.insert(it,_interval);
			return;
		}
	}
	avail.push_back(_interval);
}

/*****************************************TIM Functions****************************************************/

std::unordered_map<WholeTask*,std::list<OccupiedInterval*>>&
TaskPlan::
GetTIM() {
	return m_TIM;
}

std::list<OccupiedInterval*>
TaskPlan::
GetTaskIntervals(WholeTask* _wholeTask){
	return m_TIM[_wholeTask];
}

void
TaskPlan::
UpdateTIM(WholeTask* _wholeTask, OccupiedInterval* _interval){
	auto& plan = m_TIM[_wholeTask];
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

