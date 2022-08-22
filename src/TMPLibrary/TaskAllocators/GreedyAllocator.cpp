#include "GreedyAllocator.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"

#include "TMPLibrary/Solution/Plan.h"

/*----------------------- Construction -----------------------*/

GreedyAllocator::
GreedyAllocator() {
  this->SetName("GreedyAllocator");
}

GreedyAllocator::
GreedyAllocator(XMLNode& _node) : TaskAllocatorMethod(_node) {
  this->SetName("GreedyAllocator");

  // TODO::Parse xml node
  m_multiSolver = _node.Read("multiAgentSolver", false, "PRM", "Provide multi agent solver label.");

}


/*------------------- Allocator Interface --------------------*/

void
GreedyAllocator::
AllocateTasks() {

  Initialize();

  // TODO::Compute Allocation

  // To get the motion planning library
  //auto lib = this->GetMPLibrary();

  // To get the problem
  // auto problem = this->GetMPProblem();

  // To compute motion plan for a task
  // lib->Solve(problem,task,m_solution.get(),
  //            LRand(),this->GetNameAndLabel()+"::"+task->GetLabel());


  auto plan = this->GetPlan();
  auto decomp = plan->GetDecomposition();
  auto semanticTasks = decomp->GetMotionTasks();

  for(auto semanticTask : semanticTasks) {
    AllocateTask(semanticTask);
  }
}

/*-------------------- Helper Functions ----------------------*/

void
GreedyAllocator::
Initialize() {
  auto problem = this->GetMPProblem();
  //auto lib = this->GetMPLibrary();

  // Intiialize robot start positions
  for(auto& robot : problem->GetRobots()) {
    auto startPosition = problem->GetInitialCfg(robot.get());
    m_currentPositions[robot.get()] = startPosition;
  }

  // Initialize mp solution object if it is not yet initialized
  if(!m_initialized) {
    m_solution = std::unique_ptr<MPSolution>(new MPSolution(
                  this->GetPlan()->GetCoordinator()->GetRobot()));
  }

  cout << "m_solution" << m_solution.get() << endl;

  m_initialized = true;
}

void
GreedyAllocator::
AllocateTask(SemanticTask* _semanticTask) {

  auto lib = this->GetMPLibrary();
  auto problem = this->GetMPProblem();

  // TODO::Assign task to robot that can finish it first

  Robot* bestRobot;
  double bestCost = numeric_limits<double>::max();

  // Iterate through all robots
  for(auto& robot : problem->GetRobots()) {
    // TODO::Compute cost from robot's current position to start
    //       Use the CreateMPTask function with robot's current position as start and start of
    //       SemanticTask as the goal (_semanticTask->GetMotionTask()->GetStartConstraint())

    std::cout<< "Checking robot " << robot.get() << endl;

    auto task = CreateMPTask(robot.get(), problem->GetInitialCfg(robot.get()), _semanticTask->GetMotionTask()->GetStartConstraint());

    // compute cost of path
    //task->SetRobot(robot.get());
    //m_solution->GetPath()->Clear();

    //auto c = m_solution->GetRoadmap(this->GetPlan()->GetCoordinator()->GetRobot());

    //auto roadmap = m_solution->GetRoadmap(robot.get());
    //m_solution->SetRoadmap(robot.get(),c);

    //auto solution = new MPSolution(robot.get());
    //solution->SetRoadmap(robot.get(), lib->GetMPSolution()->GetRoadmap(robot.get()));
    m_solution->AddRobot(robot.get());
    lib->Solve(problem, task.get(),
      m_solution.get(), "BasicPRM", LRand(),
      this->GetNameAndLabel()+"::"+task->GetLabel());


    double pathCost;
    if(!m_solution->GetPath()->Cfgs().empty()){
      pathCost = m_solution->GetPath()->Length();
    } else {
      pathCost = std::numeric_limits<double>::max();
    }


    // compute cost (i.e. distance)
    //double pathCost = m_solution->GetPath()->Length();
    std::cout << "path cost = " << pathCost << endl;

    // TODO::Compute cost for robot to execute task
    //       Use the motion task inside the SemanticTask (_semanticTask->GetMotionTask())

    //if (m_solution->GetPath()->Cfgs().empty()) {
    //  lib->Solve(problem,_semanticTask->GetMotionTask().get());
    //}
    //else
    lib->Solve(problem,_semanticTask->GetMotionTask().get(),m_solution.get());

    // compute cost of this task
    double taskCost = m_solution->GetPath()->Length();
    std::cout << "task cost = " << taskCost << endl;

    double totalCost = pathCost + taskCost;


    // TODO::Assign to best robot
    if (totalCost < bestCost){
      bestCost = totalCost;
      bestRobot = robot.get();
    }

  }


  // TODO::Update plan allocation
  SaveAllocation(bestRobot,_semanticTask);

}

void
GreedyAllocator::
SaveAllocation(Robot* _robot, SemanticTask* _task) {

  if(m_debug) {
    std::cout << "Allocation "
              << _task->GetLabel()
              << " to "
              << _robot->GetLabel()
              << std::endl;
  }

  // Add the allocation to the plan
  auto plan = this->GetPlan();

  // TODO::Build out task solution object - do this much later
  plan->AddAllocation(_robot,_task);

  // TODO::Update the robot position
  auto rm = m_solution->GetRoadmap(_robot);
  auto path = m_solution->GetPath(_robot);
  auto lastVID = path->VIDs().back();
  auto lastPosition = rm->GetVertex(lastVID);
  m_currentPositions[_robot] = lastPosition;
}

std::shared_ptr<MPTask>
GreedyAllocator::
CreateMPTask(Robot* _robot, const Cfg& _start, const Cfg& _goal) {

  // Create start constraint
  auto startConstraint = std::unique_ptr<Constraint>(
        new CSpaceConstraint(_robot,_start));

  // Create goal constraint
  auto goalConstraint = std::unique_ptr<Constraint>(
        new CSpaceConstraint(_robot,_goal));

  // Create task
  auto task = std::shared_ptr<MPTask>(new MPTask(_robot));
  task->SetStartConstraint(std::move(startConstraint));
  task->AddGoalConstraint(std::move(goalConstraint));

  return task;
}

std::shared_ptr<MPTask>
GreedyAllocator::
CreateMPTask(Robot* _robot, const Cfg& _start, const Constraint* _goal) {

  // Create start constraint
  auto startConstraint = std::unique_ptr<Constraint>(
        new CSpaceConstraint(_robot,_start));

  // Create task
  auto task = std::shared_ptr<MPTask>(new MPTask(_robot));
  task->SetStartConstraint(std::move(startConstraint));
  task->AddGoalConstraint(std::move(_goal->Clone()));

  return task;
}

/*------------------------------------------------------------*/
