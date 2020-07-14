#include "SimpleMotionEvaluator.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"
#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/Solution/TaskSolution.h"

SimpleMotionEvaluator::
SimpleMotionEvaluator() {
	this->SetName("SimpleMotionEvaluator");
}

SimpleMotionEvaluator::
SimpleMotionEvaluator(XMLNode& _node) : TaskEvaluatorMethod(_node) {
	this->SetName("SimpleMotionEvaluator");
}

SimpleMotionEvaluator::
~SimpleMotionEvaluator() {}

bool 
SimpleMotionEvaluator::
Run(std::vector<WholeTask*> _wholeTasks, std::shared_ptr<TaskPlan> _plan) {

	auto plan = this->GetPlan();	
	auto decomp = plan->GetDecomposition();
	auto tasks = decomp->GetMotionTasks();
	auto problem = this->GetMPProblem();
	auto pmpl = this->GetMPLibrary();

	for(auto task : tasks) {
		// Set up storage and call motion planner
		auto mt = task->GetMotionTask();
		MPSolution* mpSolution = new MPSolution(mt->GetRobot());
		if(mt->GetStatus().is_complete()) 
			continue;
		pmpl->Solve(problem, mt.get(), mpSolution);

		// Save solution in plan
		auto solution = std::shared_ptr<TaskSolution>(new TaskSolution(task));
		solution->SetRobot(mt->GetRobot());
		solution->SetMotionSolution(mpSolution);
		plan->AddAllocation(mt->GetRobot(), task);
		plan->SetTaskSolution(task, solution);
	}

	//TODO:: MultiRobot Plans
	return true;	
}
