#include "MultiAgentMultiTaskPlanner.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/MPTask.h"

#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"
#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/WholeTask.h"

#include "Utilities/SSSP.h"

/*****************************************Constructor****************************************************/
MultiAgentMultiTaskPlanner::
MultiAgentMultiTaskPlanner(){
	this->SetName("MultiAgentMultiTaskPlanner");
}

MultiAgentMultiTaskPlanner::
MultiAgentMultiTaskPlanner(XMLNode& _node) : TMPStrategyMethod(_node){
	this->SetName("MultiAgentMultiTaskPlanner");
}

MultiAgentMultiTaskPlanner::
~MultiAgentMultiTaskPlanner(){}

void
MultiAgentMultiTaskPlanner::
Initialize(){
	TMPStrategyMethod::Initialize();
}

/******************************************Configure*****************************************************/



/*****************************************Call Method****************************************************/

void
MultiAgentMultiTaskPlanner::
PlanTasks(){
  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
	auto dummyTask = new MPTask(robot);	
	this->GetMPLibrary()->SetTask(dummyTask);
	
	//auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());

	this->GetTaskEvaluator(m_teLabel)->operator()();
}

/*****************************************TaskGraph Functions****************************************************/
