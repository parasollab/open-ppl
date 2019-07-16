#include "ITMethod.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/StateGraphs/StateGraph.h"
#include "TMPLibrary/TaskDecomposers/ITTaskBreakup.h"
#include "TMPLibrary/TaskPlan.h"

#include "Simulator/Simulation.h"
/**********************************Construction********************************/

ITMethod::
ITMethod(){
	this->SetName("ITMethod");
}

ITMethod::
ITMethod(XMLNode& _node) : TMPStrategyMethod(_node) {
	this->SetName("ITMethod");
}

/*ITMethod::
ITMethod(bool _useITs, bool _debug, std::string _dmLabel, double _connectionThreshold,
									Environment* _interactionEnvironment) :
									TMPStrategyMethod(_useITs, _debug, _dmLabel, _connectionThreshold, _interactionEnvironment){}
*/
/***********************************Call Method********************************/

void
ITMethod::
PlanTasks(){
	QueryCombinedRoadmap();
}

/***********************************Configure**********************************/
      

/********************************Combined Roadmap******************************/

void 
ITMethod::
QueryCombinedRoadmap(){
  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
	auto solution = new MPSolution(robot);
	solution->SetRoadmap(robot,this->GetStateGraph(m_sgLabel)->GetGraph());

  //TODO::Probably don't need intitial task
  auto initTask = this->GetMPProblem()->GetTasks(robot)[0];
  this->GetMPLibrary()->SetTask(initTask.get());
  this->GetMPLibrary()->Solve(this->GetMPProblem(), initTask.get(), solution, "EvaluateMapStrategy",
      LRand(), "InitTask");

  //m_solution->SetRoadmap(m_robot, m_combinedRoadmap);

  if(m_debug){
    Simulation::Get()->AddRoadmap(this->GetStateGraph(m_sgLabel)->GetGraph(),
      glutils::color(1., 0., 1., 0.2));
  }
  //Find path for each whole task in megaRoadmap
  for(auto& wholeTask: this->GetTaskPlan()->GetWholeTasks()){
    Simulation::GetStatClass()->StartClock("IT MegaRoadmap Query");
    wholeTask->m_task->SetRobot(robot);
    this->GetMPLibrary()->SetTask(wholeTask->m_task.get());
    auto& c = wholeTask->m_task->GetGoalConstraints()[0];
    c->Clone();
    this->GetMPLibrary()->Solve(this->GetMPProblem(), wholeTask->m_task.get(), solution, "EvaluateMapStrategy",
      LRand(), "PlanWholeTask");
    //Save cfg path in the handoff class
    wholeTask->m_wholePath = solution->GetPath()->Cfgs();
    Simulation::GetStatClass()->StopClock("IT MegaRoadmap Query");
    //TODO:: Need to update for multiple tasks
    Simulation::GetStatClass()->SetStat("WholePathLength", solution->GetPath()->Length());
  }
}

/*********************************Task Assignment******************************/
 
void
ITMethod:: 
AssignTasks(){
  this->GetTaskAllocator(m_taLabel)->AllocateTasks();
  /*
	TaskPlan* taskPlan = this->GetTaskPlan();
	//TODO::Abstract this and AuctionTask Function to an auction class that this calls in stead
  // Load m_unassignedTasks with the initial subtasks for all tasks.
  Simulation::GetStatClass()->StartClock("IT Task Assignment");
  for(auto& wholeTask : taskPlan->GetWholeTasks()){
    auto subtask = taskPlan->GetNextSubtask(wholeTask);
    if(subtask){
      m_unassignedTasks.push_back(subtask);
      taskPlan->SetWholeTaskOwner(subtask, wholeTask);
    }
  }
  for(auto agent : taskPlan->GetTeam()){
    agent->GetRobot()->SetVirtual(true);
  }
  // Assign all of the tasks (and their subtasks) to different agents.
  int numSubTs = 0;
  while(!m_unassignedTasks.empty()){
    std::cout << "SubTask: " << numSubTs << std::endl;
    numSubTs++;
    std::shared_ptr<MPTask> nextTask = m_unassignedTasks.front();
    auto newSubtask = AuctionTask(nextTask);
    
		m_unassignedTasks.pop_front();
    if(newSubtask){
      AddSubtask(newSubtask);
    }
  }
  for(auto agent : taskPlan->GetTeam()){
    agent->GetRobot()->SetVirtual(false);
  }
  Simulation::GetStatClass()->StopClock("IT Task Assignment");
	*/
}

void
ITMethod::
DecomposeTasks(){
	auto td = this->GetTaskDecomposer(m_tdLabel);

	Simulation::GetStatClass()->StartClock("IT Task Decomposition");
  for(auto& wholeTask : this->GetTaskPlan()->GetWholeTasks()){
    td->BreakupTask(wholeTask);
    Simulation::GetStatClass()->StopClock("IT Task Decomposition");
    Simulation::GetStatClass()->SetStat("Subtasks", wholeTask->m_subtasks.size());
  }
}

std::shared_ptr<MPTask> 
ITMethod::
AuctionTask(std::shared_ptr<MPTask> _nextTask){


  HandoffAgent* minAgent = nullptr;
  double minCost = std::numeric_limits<double>::max();

  // Generate the cost of a task for each agent
  // TODO: Thread this
  //std::vector<std::thread> costThreads(this->GetTaskPlan()->GetTeam().size());
  //auto lastAgent = GetLastAgent(m_subtaskMap[_nextTask]);
  for(auto agent : this->GetTaskPlan()->GetTeam()){
    if(m_debug){
      std::cout << "Agent capability: " << agent->GetCapability() << std::endl;
      std::cout << "Task capability: " << _nextTask->GetCapability() << std::endl;
    }
    if(agent->GetCapability() != _nextTask->GetCapability() and _nextTask->GetCapability() != "")
      continue;
    //if(agent == lastAgent)
    //  continue;
    agent->GetRobot()->SetVirtual(false);
    agent->GenerateCost(_nextTask);
    agent->GetRobot()->SetVirtual(true);
    /*auto tempThread = std::thread([agent, _nextTask](){
      agent->GenerateCost(_nextTask);
    });
    costThreads.push_back(std::move(tempThread));*/
  }
  /*for(auto& thread : costThreads){
    thread.join();
  }*/

  // Assign the task to the agent with the lowest cost.
  for(auto agent : this->GetTaskPlan()->GetTeam()){
    if(m_debug){
      std::cout << "Agent capability: " << agent->GetCapability() << std::endl;
      std::cout << "Task capability: " << _nextTask->GetCapability() << std::endl;
    }
    if(agent->GetCapability() != _nextTask->GetCapability() and _nextTask->GetCapability() != "")
      continue;
    if(m_debug){
      std::cout << agent->GetRobot()->GetLabel()
        << " has cost of "
        << agent->GetPotentialCost()
        << std::endl;
    }
    if(agent->GetPotentialCost() < minCost){
      minCost = agent->GetPotentialCost();
      minAgent = agent;
    }
  }
  if(m_debug){
    std::cout << "MinAgent: " << minAgent->GetRobot()->GetLabel() << std::endl;
  }
  // Assign subtask to min agent
  _nextTask->SetRobot(minAgent->GetRobot());
	auto wholeTask = this->GetTaskPlan()->GetWholeTask(_nextTask);
  if(!this->GetTaskPlan()->GetWholeTask(_nextTask)->m_agentAssignment.empty()){
    auto previousAgent = wholeTask->m_agentAssignment.back();
    //check if prior subtask was assigned to the same robot and merge them if so
    if(previousAgent == minAgent){
      wholeTask->m_subtaskIterator--;
      auto lastTask = wholeTask->m_subtasks[wholeTask->m_subtaskIterator-1];
      lastTask->ClearGoalConstraints();
      for(auto& constraint : _nextTask->GetGoalConstraints()){
        lastTask->AddGoalConstraint(constraint->Clone());
      }
      wholeTask->m_subtasks.erase(wholeTask->m_subtasks.begin()
                                        + wholeTask->m_subtaskIterator);
			this->GetTaskPlan()->RemoveLastDependency(_nextTask);
    }
    else{
      wholeTask->m_agentAssignment.push_back(minAgent);
			this->GetTaskPlan()->AddSubtask(minAgent,_nextTask);
    }
  }
  else{
    wholeTask->m_agentAssignment.push_back(minAgent);
		this->GetTaskPlan()->AddSubtask(minAgent,_nextTask);
  }
  // See how long this agent will take to complete the subtask and if it
  // ends in a handoff add the next subtask to the queue
  // TODO::Figure out how to keep track of time in a way accessible to everything
  double endTime = minAgent->GetTaskTime();// + m_currentTime;
  std::shared_ptr<MPTask> newSubtask = this->GetTaskPlan()->GetNextSubtask(wholeTask);
  if(newSubtask){
    this->GetTaskPlan()->SetWholeTaskOwner(newSubtask,wholeTask);
    newSubtask->SetEstimatedStartTime(endTime);
		//NEW
		this->GetTaskPlan()->AddDependency(_nextTask,newSubtask);
  }
  return newSubtask;
}


/*********************************Helper Functions******************************/

void 
ITMethod::
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
