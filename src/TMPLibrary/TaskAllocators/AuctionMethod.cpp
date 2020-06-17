#include "AuctionMethod.h"

#include "TMPLibrary/TaskPlan.h"

#include "Simulator/Simulation.h"
#include "Utilities/MetricUtils.h"
/*------------------------------ Construction --------------------------------*/

AuctionMethod::
AuctionMethod() {
	this->SetName("AuctionMethod");
}

AuctionMethod::
AuctionMethod(XMLNode& _node) : TaskAllocatorMethod(_node){
	this->SetName("AuctionMethod");
}

void
AuctionMethod::
AllocateTasks(){
	this->GetMPLibrary()->GetStatClass()->StartClock("Auction Method");
	/*for(auto& wholeTask : this->GetTaskPlan()->GetWholeTasks()){
    auto subtask = this->GetTaskPlan()->GetNextSubtask(wholeTask);
		if(subtask){
			m_unassignedTasks.push_back(subtask);
			this->GetTaskPlan()->SetWholeTaskOwner(subtask, wholeTask);
		}
	}*/

	for(auto agent : this->GetTaskPlan()->GetTeam()){
    agent->GetRobot()->SetVirtual(true);
  }
	for(auto& wholeTask : this->GetTaskPlan()->GetWholeTasks()){
		AuctionSubtasks(wholeTask);
	}


	this->GetTaskPlan()->GetStatClass()->StopClock("Auction Method");
}



/***********************************Helpers**********************************/
void 
AuctionMethod::
AuctionSubtasks(WholeTask* _wholeTask){
	m_unassignedTasks.clear();
	auto subtask = this->GetTaskPlan()->GetNextSubtask(_wholeTask);
	if(!subtask)
		return;
	m_unassignedTasks.push_back(subtask);
	this->GetTaskPlan()->SetWholeTaskOwner(subtask,_wholeTask);

	while(!m_unassignedTasks.empty()){
    std::shared_ptr<MPTask> nextTask = m_unassignedTasks.front();
    auto newSubtask = AuctionTask(nextTask);
    
		m_unassignedTasks.pop_front();
    if(newSubtask){
      AddSubtask(newSubtask);
    }
	}
}

std::shared_ptr<MPTask> 
AuctionMethod::
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
			minAgent->SavePotentialPath();
    }
  }
  else{
    wholeTask->m_agentAssignment.push_back(minAgent);
		this->GetTaskPlan()->AddSubtask(minAgent,_nextTask);
		minAgent->SavePotentialPath();
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


void 
AuctionMethod::
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
