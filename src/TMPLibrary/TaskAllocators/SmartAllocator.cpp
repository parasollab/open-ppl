#include "SmartAllocator.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"

#include "TMPLibrary/Solution/Plan.h"

/*----------------------- Construction -----------------------*/

SmartAllocator::
SmartAllocator() {
  this->SetName("SmartAllocator");
}

SmartAllocator::
SmartAllocator(XMLNode& _node) : TaskAllocatorMethod(_node) {
  this->SetName("SmartAllocator");

  // TODO::Parse xml node
  m_singleSolver = _node.Read("singleAgentSolver", false, "BasicPRM", "Provide single agent solver label.");

}


/*------------------- Allocator Interface --------------------*/

void
SmartAllocator::
AllocateTasks() {

  Initialize();

  // Compute Allocation

  auto plan = this->GetPlan();
  auto decomp = plan->GetDecomposition();
  auto semanticTasks = decomp->GetMotionTasks();

  AllocateTask(semanticTasks);

  //for(auto semanticTask : semanticTasks) {
  //  AllocateTask(semanticTask);
  //}
}

/*-------------------- Helper Functions ----------------------*/

void
SmartAllocator::
Initialize() {
  auto problem = this->GetMPProblem();

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

    auto startPosition = problem->GetInitialCfg(robot.get());
    m_currentPositions[robot.get()] = startPosition;
    m_solution->AddRobot(robot.get());
  }

  m_initialized = true;
}

void
SmartAllocator::
AllocateTask(vector<SemanticTask*> _semanticTasks) {

  auto lib = this->GetMPLibrary();
  auto problem = this->GetMPProblem();
  
  // determine how many robots/tasks we have so we know the size of the cost matrix
  int n = problem->NumRobots() -1; // subtract 1 to exclude the coordinator 
  int m = _semanticTasks.size();

  // create an nxm array. n = num robots, m = num tasks
  //vector<vector<int>> costMatrix(n, vector<int>(m));

  double costMatrix[n][m];

  // Iterate through each robot/task and get cost assignments for each of them 

  int i = 0;
  int j = 0;
  // Iterate through all robots
  auto plan = this->GetPlan();
  auto coordinator = plan->GetCoordinator();
  for(auto& robot : problem->GetRobots()) {

    if(robot.get() == coordinator->GetRobot())
      continue;
    j = 0;

    for(auto& semanticTask : _semanticTasks) {
      
     
      auto task = CreateMPTask(robot.get(), m_currentPositions[robot.get()], semanticTask->GetMotionTask()->GetStartConstraint());
    
      task->SetRobot(robot.get());
  
      // solve the "getting there" task 
      lib->Solve(problem, task.get(), m_solution.get(), m_singleSolver, LRand(), this->GetNameAndLabel()+"::"+task->GetLabel());

      // compute cost of path
      double pathCost;
      if(!m_solution->GetPath()->Cfgs().empty()){ // only get the cost if we found a valid path  
        pathCost = m_solution->GetPath()->Length();
      } else {
        pathCost = std::numeric_limits<double>::max(); // otherwise, assign some huge cost for this task
      }

      // Compute cost for robot to execute task
      lib->Solve(problem,semanticTask->GetMotionTask().get(),m_solution.get());

      // compute cost of this task
      double taskCost = m_solution->GetPath()->Length();

      double totalCost = pathCost + taskCost;

      
      if (m_debug) std::cout << "the cost to assign robot " << i << " to task " << j << " is "<< totalCost << endl;

      costMatrix[i][j] = totalCost;
      
      j++;

    }

    i++;

  }


  if (m_debug) { 
  //print matrix 
  std::cout<< "the cost matrix is " << endl;
    for(int k = 0; k < n; k++) {
      for(int l = 0; l < m; l++) {
        std::cout << costMatrix[k][l] << " ";
      }
      std::cout << "\n";
    }
  }

  // once we have the cost matrix, pass the matrix into the munkres assignment algorithm


  // Update plan allocation
  //SaveAllocation(bestRobot,_semanticTask);

}

void
SmartAllocator::
SaveAllocation(Robot* _robot, SemanticTask* _task) {

  if(true) {
    std::cout << "Allocation "
              << _task->GetLabel()
              << " to "
              << _robot->GetLabel()
              << std::endl;
  }

  // Add the allocation to the plan
  auto plan = this->GetPlan();

  // TODO::Build our task solution object - do this much later
  plan->AddAllocation(_robot,_task);

  // Update the robot position
  auto rm = m_solution->GetRoadmap(_robot);
  auto path = m_solution->GetPath(_robot);
  auto lastVID = path->VIDs().back();
  auto lastPosition = rm->GetVertex(lastVID);
  m_currentPositions[_robot] = lastPosition;
}

std::shared_ptr<MPTask>
SmartAllocator::
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
SmartAllocator::
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
