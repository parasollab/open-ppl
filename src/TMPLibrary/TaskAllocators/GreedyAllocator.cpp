#include "GreedyAllocator.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"

#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/Solution/TaskSolution.h"

/*----------------------- Construction -----------------------*/

GreedyAllocator::
GreedyAllocator() {
  this->SetName("GreedyAllocator");
}

GreedyAllocator::
GreedyAllocator(XMLNode& _node) : TaskAllocatorMethod(_node) {
  this->SetName("GreedyAllocator");

  // TODO::Parse xml node
  m_singleSolver = _node.Read("singleAgentSolver", false, "BasicPRM", "Provide single agent solver label.");

  m_clearAfterInitializing = _node.Read("reset",false,m_clearAfterInitializing,
      "Flag to reset all existing allocations after initializing.");

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

  // Initialize mp solution object if it is not yet initialized
  if(!m_initialized) {
    m_solution = std::unique_ptr<MPSolution>(new MPSolution(
                  this->GetPlan()->GetCoordinator()->GetRobot()));
  }

  
  // Intialize robot start positions
  auto plan = this->GetPlan();
  auto coordinator = plan->GetCoordinator();
  for(auto& robot : problem->GetRobots()) {
    if(robot.get() == coordinator->GetRobot())
      continue;

    Cfg startPosition(robot.get());
    double nextFreeTime = 0.;

    auto allocs = plan->GetAllocations(robot.get());
    if(allocs.empty()) {
      startPosition = problem->GetInitialCfg(robot.get());
    }
    else {
      auto task = allocs.back();
      auto sol = plan->GetTaskSolution(task);
      auto vid = sol->GetPath()->VIDs().back();
      startPosition = sol->GetMotionSolution()->GetRoadmap(
                        robot.get())->GetVertex(vid);

      auto startTime = sol->GetStartTime();
      auto length = sol->GetPath()->Length();
      // TODO::This should really be timesteps not length
      nextFreeTime = startTime + length;

      if(m_clearAfterInitializing)
        plan->ClearAllocations(robot.get());
    }
    
    m_currentPositions[robot.get()] = startPosition;
    m_nextFreeTime[robot.get()] = nextFreeTime; 

    m_solution->AddRobot(robot.get());
  }

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
  std::unique_ptr<Path> bestPath;

  // Iterate through all robots
  auto plan = this->GetPlan();
  auto coordinator = plan->GetCoordinator();
  for(auto& robot : problem->GetRobots()) {
    if(robot.get() == coordinator->GetRobot())
      continue;


    // Use the CreateMPTask function to create a task with robot's current position as start and start of
    // SemanticTask as the goal (_semanticTask->GetMotionTask()->GetStartConstraint())
    // This task represents the robot getting to the start of the actual task 

    auto task = CreateMPTask(robot.get(), m_currentPositions[robot.get()], _semanticTask->GetMotionTask()->GetStartConstraint());
 
    
    task->SetRobot(robot.get());

    //lib->SetMPSolution(m_solution.get());
    //m_solution->GetPath()->Clear();

    // solve the "getting there" task 
    lib->Solve(problem, task.get(), m_solution.get(), m_singleSolver, LRand(), this->GetNameAndLabel()+"::"+task->GetLabel());

    // compute cost of path
    double pathCost;
    auto path = m_solution->GetPath(robot.get());

    if(path->VIDs().empty())
      continue;

    auto rm = m_solution->GetRoadmap(robot.get());
    auto pathCopy = std::unique_ptr<Path>(new Path(rm));
    pathCost = path->Length();
    *(pathCopy.get()) = *path;

    // compute cost (i.e. distance)
    std::cout << "path cost = " << pathCost << endl;

    // Compute cost for robot to execute task
    // Use the motion task inside the SemanticTask (_semanticTask->GetMotionTask())

    auto mainTask = _semanticTask->GetMotionTask().get();
    mainTask->SetRobot(robot.get());    

    lib->Solve(problem,mainTask,m_solution.get());

    path = m_solution->GetPath();
    if(path->VIDs().empty())
      continue;

    *(pathCopy.get()) += path->VIDs();    

    // compute cost of this task
    double taskCost = m_solution->GetPath()->Length();
    std::cout << "task cost = " << taskCost << endl;
    
    double totalCost = pathCost + taskCost;

    double estimatedCompletion = totalCost + m_nextFreeTime[robot.get()];

    std::cout << "estimated completion = " << estimatedCompletion << std::endl;

    // Assign to best robot
    if (estimatedCompletion < bestCost){
      bestCost = estimatedCompletion;
      bestRobot = robot.get();
      bestPath = std::move(pathCopy);
    }
  }

  // Update plan allocation
  SaveAllocation(bestRobot,_semanticTask,std::move(bestPath));

}

void
GreedyAllocator::
SaveAllocation(Robot* _robot, SemanticTask* _task, std::unique_ptr<Path> _path) {

  if(true) {
    std::cout << "Allocation "
              << _task->GetLabel()
              << " to "
              << _robot->GetLabel()
              << std::endl;
  }

  // Update the robot position
  auto rm = m_solution->GetRoadmap(_robot);
  auto lastVID = _path->VIDs().back();
  auto lastPosition = rm->GetVertex(lastVID);
  m_currentPositions[_robot] = lastPosition;
  auto startTime = m_nextFreeTime[_robot];
  m_nextFreeTime[_robot] += _path->Length();

  // Add the allocation to the plan
  auto plan = this->GetPlan();
  plan->AddAllocation(_robot,_task);

  // TODO::Build our task solution object - do this much later
  auto sol = std::shared_ptr<TaskSolution>(new TaskSolution(_task));
  sol->SetRobot(_robot);
  sol->SetMotionSolution(m_solution.get());
  sol->SetPath(std::move(_path));
  sol->SetStartTime(startTime);

  plan->SetTaskSolution(_task,sol);
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
  auto goal = _goal->Clone();
  goal->SetRobot(_robot);
  task->AddGoalConstraint(std::move(goal));

  return task;
}

/*------------------------------------------------------------*/
