#include "HandoffAgent.h"

#include <limits>
#include <unordered_map>
#include <algorithm>
#include "ConfigurationSpace/Cfg.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "Behaviors/Controllers/ICreateController.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include "nonstd/container_ops.h"
#include "nonstd/numerics.h"
#include "nonstd/timer.h"
#include "nonstd/io.h"

#include "Simulator/Simulation.h"


/*------------------------------ Construction --------------------------------*/

HandoffAgent::
HandoffAgent(Robot* const _r) : PathFollowingAgent(_r) {
}


HandoffAgent::
HandoffAgent(Robot* const _r, XMLNode& _node)
  : PathFollowingAgent(_r, _node) {
  // Parse XML parameters.
}


HandoffAgent::
~HandoffAgent() {
  // Ensure agent is properly torn down.
  Uninitialize();
}

/*---------------------------- Simulation Interface --------------------------*/

void
HandoffAgent::
Initialize() {
  PathFollowingAgent::Initialize();
}

void
HandoffAgent::
InitializeRoadmap() {
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*m_robot->GetMPProblem()));
  // TODO: Change the workfunction to work with BasicPRM (parameterize it)
  WorkFunction(problemCopy);
}

void
HandoffAgent::
SetParentAgent(Agent* const _parent) {
  m_parentAgent = _parent;
}


bool
HandoffAgent::
IsChild() const noexcept {
  return true;
}


void 
HandoffAgent::
GenerateCost(std::shared_ptr<MPTask> const _task) {
  // Save current state in case the robot has not finished its current task.
  // TODO: Find way to check if initial task is completed if secondary task is
  // assigned (maybe change m_task to a queue - m_tasks)
  auto currentTask = GetTask(); 
  auto currentPath = m_path;

  // TODO: Create cost functions for the path (adjust edge weights according to
  // metric and agent type).
  m_potentialCost = 0.0;
  if(!currentPath.empty())
    m_potentialCost = m_solution->GetPath()->Length();
  
  // Compute cost to travel from current position to start constraint
  Cfg position;
  // If the robot currently has a path, set its position to the end of that path
  // Otherwise, the robot will start the task from its current position
  if(!currentPath.empty())
    position = m_path.back();
  else
    position = m_robot->GetDynamicsModel()->GetSimulatedState();

  auto start = std::unique_ptr<CSpaceConstraint>(
      new CSpaceConstraint(m_robot, position));
  std::shared_ptr<MPTask> setupTask(new MPTask(m_robot));
  setupTask->SetStartConstraint(std::move(start));
  std::unique_ptr<Constraint> setupStart(_task->GetStartConstraint()->Clone());
  setupTask->AddGoalConstraint(std::move(setupStart));
  
  SetTask(setupTask);
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*m_robot->GetMPProblem()));
  WorkFunction(problemCopy);
  auto setupPath = m_path;
  m_potentialCost += m_solution->GetPath()->Length();

  // Compute cost to travel from start constraint to goal constraint
  SetTask(_task);
  std::shared_ptr<MPProblem> problemCopyDos(new MPProblem(*m_robot->GetMPProblem()));
  WorkFunction(problemCopyDos);
  auto goalPath = m_path;
  m_potentialCost += m_solution->GetPath()->Length();
  
  // Save the computed paths in case this robot is selected to perform the task.
  m_potentialPath = currentPath;
  m_potentialPath.insert(m_potentialPath.end(), setupPath.begin(), setupPath.end());
  m_potentialPath.insert(m_potentialPath.end(), goalPath.begin(), goalPath.end());

  // Restore the task/path state to currentTask/currentPath
  SetTask(currentTask);
  m_path = currentPath;
}

double
HandoffAgent::
GetPotentialCost() const {
  return m_potentialCost;
}


double 
HandoffAgent::
GetTaskTime() const {
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();
  const Cfg position = m_robot->GetDynamicsModel()->GetSimulatedState();
  std::cout << "Position: " << position.PrettyPrint() << std::endl;
  std::cout << "Position: "
            << m_robot->GetDynamicsModel()->GetSimulatedState().PrettyPrint()
            << std::endl;

  // Use the controller and dynamics model to generate an ideal course for this
  // path.
  const auto& path = m_solution->GetPath()->Cfgs();
  auto controller  = m_robot->GetController();
  auto dynamics    = m_robot->GetDynamicsModel();
  auto dm          = m_library->GetDistanceMetric(m_waypointDm);

  double numSteps = 0;

  for(size_t i = 1; i < path.size(); ++i) {
    // Get the next pair of configurations.
    Cfg         current  = path[i - 1];
    const auto& waypoint = path[i];

    // While current is too far from way point, use the controller to generate
    // a control and test it with the dynamics model.
    while(dm->Distance(current, waypoint) > m_waypointThreshold) {

      // Apply the next control.
      Control nextControl = (*controller)(current, waypoint, timeRes);
      //std::cout << "Controls being applied: " << nextControl << std::endl;
      current = dynamics->Test(current, nextControl, timeRes);
      //std::cout << "Current: " << current.PrettyPrint() << std::endl;
      numSteps += 1;
    }
  }

  position.ConfigureRobot();
  dynamics->SetSimulatedState(position);
  std::cout << "Position: "
            << m_robot->GetDynamicsModel()->GetSimulatedState().PrettyPrint()
            << std::endl;
  
  return numSteps * timeRes;

}
/*------------------------------ Helpers -------------------------------------*/

void
HandoffAgent::
WorkFunction(std::shared_ptr<MPProblem> _problem) {
  // TODO: Stop trying to plan if it takes longer than t_max
  // TODO: Parameterize this later to avoid hardcoding to LazyPRM
  std::cout << m_robot->GetLabel()
            << " STARTING PLANNING LAZYQUERY"
            << std::endl;
  // Set the copy of this robot to virtual.
  auto currentRobot = _problem->GetRobot(m_robot->GetLabel());
  currentRobot->SetVirtual(true);

  // Create a task for the parent robot copy (because this is a shared roadmap
  // method).
  auto parentRobot = m_parentAgent->GetRobot();
  auto parentCopyRobot = _problem->GetRobot(parentRobot->GetLabel());
  GetTask()->SetRobot(parentCopyRobot);
  std::cout << "Calling Solve for " << m_robot->GetLabel() <<  std::endl;
  std::cout << "Currently at: " << m_robot->GetDynamicsModel()->GetSimulatedState() << std::endl;
  m_solution->GetPath()->Clear();

  // Set the solution for appending with the parent copy.
  m_solution->SetRobot(parentCopyRobot);

  // Solve for the plan.
  std::cout << "Calling Solve for " << m_robot->GetLabel() << std::endl;

  m_library->Solve(_problem.get(), GetTask().get(), m_solution.get(), "LazyPRM",
      LRand(), "LazyCollisionAvoidance");

  // Reset the modified states.
  GetTask()->SetRobot(m_robot);
  m_solution->SetRobot(m_robot);

  // Extract the path for this robot.
  m_pathIndex = 0;
  m_path = m_solution->GetPath()->Cfgs();

  std::cout << m_path << std::endl;

  // Throw if PMPL failed to generate a solution.
  // TODO: Determine what to do when failing to produce a solution.
  if(m_path.empty())
    throw RunTimeException(WHERE, "PMPL failed to produce a solution.");

  std::cout << m_robot->GetLabel() << " DONE PLANNING LAZYQUERY" << std::endl;

  m_pathVisualID = Simulation::Get()->AddPath(m_path, glutils::color::red);
  m_planning = false;
}


bool
HandoffAgent::
SelectTask(){
  //TODO: Figure out if we still need this
  //m_parentAgent->AssignTask(this);
  return GetTask().get();
}


void
HandoffAgent::
ExecuteControls(const ControlSet& _c, const size_t _steps) {
  this->Agent::ExecuteControls(_c, _steps);

  if(_c.size() > 1)
    throw RunTimeException(WHERE,
        "We are assuming that only one control will be passed in at a time.");

  // Update odometry tracking.
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();
  if(_c.size())
    m_distance += _steps * timeRes * nonstd::magnitude<double>(_c[0].GetForce());
}
