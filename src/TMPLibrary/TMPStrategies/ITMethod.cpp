#include "ITMethod.h"

#include "TMPLibrary/TaskDecomposers/ITTaskBreakup.h"
#include "TMPLibrary/TaskPlan.h"

#include "Simulator/Simulation.h"
/**********************************Construction********************************/

ITMethod::
ITMethod(XMLNode& _node) : TMPStrategyMethod(_node) {}

/*ITMethod::
ITMethod(bool _useITs, bool _debug, std::string _dmLabel, double _connectionThreshold,
									Environment* _interactionEnvironment) :
									TMPStrategyMethod(_useITs, _debug, _dmLabel, _connectionThreshold, _interactionEnvironment){}
*/
/***********************************Call Method********************************/

TaskPlan* 
ITMethod::
PlanTasks(MPLibrary* _library, vector<HandoffAgent*> _agents,
          vector<std::shared_ptr<MPTask>> _tasks){

	TMPStrategyMethod::PlanTasks(_library, _agents, _tasks);

	//LoadAgents

	this->GetTaskPlan()->GenerateDummyAgents();

	GenerateITs();

	CreateCombinedRoadmap();

	QueryCombinedRoadmap();

	CopyRobotTypeRoadmaps();

	DecomposeTasks();

	return AssignTasks();
}

/***********************************Configure**********************************/
      

/********************************Combined Roadmap******************************/

void 
ITMethod::
QueryCombinedRoadmap(){
  //TODO::Probably don't need intitial task
  auto initTask = m_robot->GetMPProblem()->GetTasks(m_robot)[0];
  m_library->SetTask(initTask.get());

  m_library->Solve(m_robot->GetMPProblem(), initTask.get(), m_solution.get(), "EvaluateMapStrategy",
      LRand(), "InitTask");

  //m_solution->SetRoadmap(m_robot, m_combinedRoadmap);

  if(m_debug){
    Simulation::Get()->AddRoadmap(m_combinedRoadmap,
      glutils::color(1., 0., 1., 0.2));
  }
  //Find path for each whole task in megaRoadmap
  for(auto& wholeTask: this->GetTaskPlan()->GetWholeTasks()){
    Simulation::GetStatClass()->StartClock("IT MegaRoadmap Query");
    wholeTask->m_task->SetRobot(m_robot);
    m_library->SetTask(wholeTask->m_task.get());
    auto& c = wholeTask->m_task->GetGoalConstraints()[0];
    c->Clone();
    m_library->Solve(m_robot->GetMPProblem(), wholeTask->m_task.get(), m_solution.get(), "EvaluateMapStrategy",
      LRand(), "PlanWholeTask");
    //Save cfg path in the handoff class
    wholeTask->m_wholePath = m_solution->GetPath()->Cfgs();
    Simulation::GetStatClass()->StopClock("IT MegaRoadmap Query");
    //TODO:: Need to update for multiple tasks
    Simulation::GetStatClass()->SetStat("WholePathLength", m_solution->GetPath()->Length());
  }
}

/*********************************Task Assignment******************************/
 
TaskPlan*
ITMethod:: 
AssignTasks(){
	TaskPlan* taskPlan = new TaskPlan();
	//TODO::Abstract this and AuctionTask Function to an auction class that this calls in stead
  // Load m_unassignedTasks with the initial subtasks for all tasks.
  Simulation::GetStatClass()->StartClock("IT Task Assignment");
  for(auto& wholeTask : this->GetTaskPlan()->GetWholeTasks()){
    auto subtask = this->GetTaskPlan()->GetNextSubtask(wholeTask);
    if(subtask){
      m_unassignedTasks.push_back(subtask);
      this->GetTaskPlan()->SetWholeTaskOwner(subtask, wholeTask);
    }
  }
  for(auto agent : this->GetTaskPlan()->GetTeam()){
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
  for(auto agent : this->GetTaskPlan()->GetTeam()){
    agent->GetRobot()->SetVirtual(false);
  }
  Simulation::GetStatClass()->StopClock("IT Task Assignment");
	
	return taskPlan;
}

void
ITMethod::
DecomposeTasks(){
  
	ITTaskBreakup tb(m_robot);

  for(auto& wholeTask : this->GetTaskPlan()->GetWholeTasks()){
    tb.BreakupTask(wholeTask);
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
CopyRobotTypeRoadmaps(){
  for(auto agent : this->GetTaskPlan()->GetTeam()){
    //Copy corresponding capability roadmap into agent
    auto graph = m_capabilityRoadmaps[agent->GetCapability()];
    auto g = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(agent->GetRobot());
    *g = *graph;
    g->SetRobot(agent->GetRobot());
    agent->SetRoadmapGraph(g);

    // Write the capability map.
    graph->Write(agent->GetRobot()->GetLabel() + ".map",
        m_robot->GetMPProblem()->GetEnvironment());
  }
}

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
