#include "SimpleMotionEvaluator.h"

#include "ConfigurationSpace/Formation.h"

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
Run(Plan* _plan) {

  Plan* plan;

  if(_plan)
    plan = _plan;
  else
    plan = this->GetPlan();

	auto decomp = plan->GetDecomposition();
	auto tasks = decomp->GetMotionTasks();
	auto groupTasks = decomp->GetGroupMotionTasks();
	auto problem = this->GetMPProblem();
	auto pmpl = this->GetMPLibrary();

	// Call motion planner for motion tasks
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

  //Temp mpsolution
  MPSolution* tempSol;

	// Call motion planner for group motion tasks
	for(auto task : groupTasks) {
		// Set up storage and call motion planner
		auto gt = task->GetGroupMotionTask();
		MPSolution* mpSolution = new MPSolution(gt->GetRobotGroup());
		pmpl->Solve(problem, gt.get(), mpSolution);

    //Temp formation stuff
    tempSol = mpSolution;

		// Save solution in plan
		auto solution = std::shared_ptr<TaskSolution>(new TaskSolution(task));
		solution->SetRobotGroup(gt->GetRobotGroup());
		solution->SetMotionSolution(mpSolution);
		plan->AddAllocation(gt->GetRobotGroup(), task);
		plan->SetTaskSolution(task, solution);
	}

  //Temp test of Formation Code 
  std::vector<Robot*> robots; 

  for(auto& robot : this->GetMPProblem()->GetRobots()) {
    if(robot->GetLabel() == "coordinator")
      continue;
    robots.push_back(robot.get());
  }

  auto body1 = robots[0]->GetMultiBody()->GetBody(0);
  auto body2 = robots[1]->GetMultiBody()->GetBody(0);

  Formation::FormationConstraint constraint;
  constraint.dependentRobot = robots[1];
  constraint.referenceRobot = robots[0];
  constraint.dependentBody = body2;
  constraint.referenceBody = body1;
  

  Transformation transformation;
  constraint.transformation = transformation;

  std::unordered_map<MultiBody*,Formation::FormationConstraint> constraintMap;
  constraintMap[robots[1]->GetMultiBody()] = constraint;

  Formation formation(robots,robots[0],constraintMap);

  auto groupCfg = tempSol->GetGroupRoadmap()->GetVertex(0);
  std::cout << groupCfg << std::endl;
 
  groupCfg.AddFormation(&formation);

  auto env = this->GetMPProblem()->GetEnvironment();
  
  groupCfg.GetRandomGroupCfg(env);
  std::cout << groupCfg << std::endl;

	return true;	
}
