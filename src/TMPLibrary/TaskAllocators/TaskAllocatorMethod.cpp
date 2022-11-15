#include "TaskAllocatorMethod.h"

#include "Behaviors/Agents/Coordinator.h"
#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/Solution/TaskSolution.h"
/*------------------------------ Construction --------------------------------*/

TaskAllocatorMethod::
TaskAllocatorMethod(XMLNode& _node) : TMPBaseObject(_node) {
  m_clearAfterInitializing = _node.Read("reset",false,m_clearAfterInitializing,
      "Flag to reset all existing allocations after initializing.");

}

void
TaskAllocatorMethod::
AllocateTasks() {}

void
TaskAllocatorMethod::
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
TaskAllocatorMethod::
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

