#include "ITMethod.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/StateGraphs/StateGraph.h"
#include "TMPLibrary/TaskAllocators/TaskAllocatorMethod.h"
#include "TMPLibrary/TaskDecomposers/ITTaskBreakup.h"
#include "TMPLibrary/TaskPlan.h"

#include "Simulator/Simulation.h"
#include "Utilities/MetricUtils.h"
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

  if(m_debug and Simulation::Get()){
    Simulation::Get()->AddRoadmap(this->GetStateGraph(m_sgLabel)->GetGraph(),
      glutils::color(1., 0., 1., 0.2));
  }
  //Find path for each whole task in megaRoadmap
  for(auto& wholeTask: this->GetTaskPlan()->GetWholeTasks()){
    this->GetTaskPlan()->GetStatClass()->StartClock("IT MegaRoadmap Query");
    wholeTask->m_task->SetRobot(robot);
    this->GetMPLibrary()->SetTask(wholeTask->m_task.get());
    auto& c = wholeTask->m_task->GetGoalConstraints()[0];
    c->Clone();
    this->GetMPLibrary()->Solve(this->GetMPProblem(), wholeTask->m_task.get(), solution, "EvaluateMapStrategy",
      LRand(), "PlanWholeTask");
    //Save cfg path in the handoff class
    wholeTask->m_wholePath = solution->GetPath()->Cfgs();
    this->GetTaskPlan()->GetStatClass()->StopClock("IT MegaRoadmap Query");
    //TODO:: Need to update for multiple tasks
    this->GetTaskPlan()->GetStatClass()->SetStat("WholePathLength", solution->GetPath()->Length());
  }
}

/*********************************Task Assignment******************************/
 
void
ITMethod:: 
AssignTasks(){
  this->GetTaskAllocator(m_taLabel)->AllocateTasks();
}

void
ITMethod::
DecomposeTasks(){
	auto td = this->GetTaskDecomposer(m_tdLabel);

  for(auto& wholeTask : this->GetTaskPlan()->GetWholeTasks()){
		this->GetTaskPlan()->GetStatClass()->StartClock("IT Task Decomposition");
    td->BreakupTask(wholeTask);
    this->GetTaskPlan()->GetStatClass()->StopClock("IT Task Decomposition");
    this->GetTaskPlan()->GetStatClass()->SetStat("Subtasks", wholeTask->m_subtasks.size());
  }
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
