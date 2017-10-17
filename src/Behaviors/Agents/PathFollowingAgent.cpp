#include "PathFollowingAgent.h"

#include <limits>

#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/HardwareInterfaces/HardwareInterface.h"


/*------------------------------ Construction --------------------------------*/

PathFollowingAgent::
PathFollowingAgent(Robot* const _r) : Agent(_r) { }

PathFollowingAgent::
~PathFollowingAgent() {
  // Ensure agent is properly torn down.
  PathFollowingAgent::Uninitialize();
}

/*------------------------------ Agent Interface -----------------------------*/

void
PathFollowingAgent::
Initialize() {
  if(m_initialized)
    return;
  m_initialized = true;

  // Get problem info.
  auto problem = m_robot->GetMPProblem();
  const std::string& xmlFile = problem->GetXMLFilename();

  // Initialize the agent's planning library.
  m_library = new MPLibrary(xmlFile);

  /// @TODO Choose the task rather than just taking the first one.
  auto task = problem->GetTasks().front();

  // Create a new solution object to hold a plan for this agent.
  auto solution = new MPSolution(task->GetRobot());

  // Use the planning library to find a path.
  m_library->Solve(problem, task, solution);

  m_hardwareController = new ICreateController(m_robot, 1,1);
  // Extract the path from the solution.
  m_path = solution->GetPath()->Cfgs();
  delete solution;
}


void 
PathFollowingAgent::
UpdateOdometry(const double& _x, const double& _y, const double& _angle) {
  m_odometry[0] = _x;
  m_odometry[1] = _y;
  m_odometry[2] += _angle;

  //If angle is greater than 2pi then subtract 2pi from the angle.
  if(m_odometry[2] >= 2*M_PI)
    m_odometry[2] = m_odometry[2] - 2*M_PI;
  //If angle is less than 2pi then add 2pi to the angle.
  else if(m_odometry[2] <= -2*M_PI)
    m_odometry[2] = m_odometry[2] + 2*M_PI;
}


void
PathFollowingAgent::
Step(const double _dt) {
  Initialize();

  // Do nothing if there are no unvisited points left.
  if(m_pathIndex >= m_path.size())
    return;

  if(m_debug)
    std::cout << "Approaching waypoint " << m_pathIndex << " / "
              << m_path.size() - 1 << ".\n";

  // Get the current configuration.
  const Cfg current = m_robot->GetDynamicsModel()->GetSimulatedState();

  // We consider the robot to have reached the next subgoal if it is within a
  // threshold distance. Advance the path index until the next subgoal is
  // at least one threshold away.
  auto dm = m_library->GetDistanceMetric("euclidean");
  const double threshold = .05;

  double distance = dm->Distance(current, m_path[m_pathIndex]);

  if(m_debug)
    std::cout << "\tDistance from current configuration: "
              << distance << "/" << threshold
              << std::endl;

  while(distance < threshold and m_pathIndex < m_path.size()) {
    if(m_debug)
      std::cout << "\tReached waypoint " << m_pathIndex << " at "
                << distance << "/" << threshold << std::endl
                << "Waypoint = " << m_path[m_pathIndex] << std::endl;

    // Move to next cfg in path since the distance is within the threshold.
    ++m_pathIndex;

    // Break if we try to go beyond the path's end. Necessary, as calculating
    // the distance on an undefined cfg will crash some systems.
    if(m_pathIndex >= m_path.size())
      break;

    distance = dm->Distance(current, m_path[m_pathIndex]);
  }

  // If we hit the end, return.
  if(m_pathIndex >= m_path.size()) {
    if(m_debug)
      std::cout << "Reached the end of the path." << std::endl;

    // Warning: Halt() doesn't respect the dynamics of the simulation and is
    // only to be used for visual verification of the path in the simulator.
    this->Halt();
    return;
  }

  // Otherwise, execute the control that is nearest to the desired force.
  auto bestControl = m_robot->GetController()->operator()(current,
      m_path[m_pathIndex], _dt);
  bestControl.Execute();

  // If there is a hardware robot attached to our simulation, send it the
  // commands also.
  auto hardwareControl = m_hardwareController->operator()(current, m_path[m_pathIndex], m_odometry[2]);
  auto hardwareInterface = m_robot->GetHardwareInterface();
  if(hardwareInterface)
    hardwareInterface->EnqueueCommand({hardwareControl}, _dt);
}


void
PathFollowingAgent::
Uninitialize() {
  if(!m_initialized)
    return;
  m_initialized = false;

  m_path.clear();
  m_pathIndex = 0;
  delete m_library;
}

/*----------------------------------------------------------------------------*/
