#include "PathFollowingAgent.h"

#include <limits>

#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/HardwareInterfaces/QueuedHardwareInterface.h"


/*------------------------------ Construction --------------------------------*/

PathFollowingAgent::
PathFollowingAgent(Robot* const _r) : Agent(_r) { }


PathFollowingAgent::
PathFollowingAgent(Robot* const _r, const PathFollowingAgent& _a) : Agent(_r, _a)
{ }


PathFollowingAgent::
PathFollowingAgent(Robot* const _r, XMLNode& _node) : Agent(_r) {
  // Currently there are no parameters. Parse XML options here.
}


std::unique_ptr<Agent>
PathFollowingAgent::
Clone(Robot* const _r) const {
  return std::unique_ptr<PathFollowingAgent>(new PathFollowingAgent(_r, *this));
}


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

  /// @TODO Choose the task intelligently rather than just taking the first one.
  auto task = problem->GetTasks(m_robot).front().get();
  this->SetTask(task);

  // Create a new solution object to hold a plan for this agent.
  auto solution = new MPSolution(m_robot);

  // Use the planning library to find a path.
  m_library->Solve(problem, task, solution);

  // Extract the path from the solution.
  m_path = solution->GetPath()->Cfgs();
  delete solution;
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

  auto hardwareInterface = static_cast<QueuedHardwareInterface*>(m_robot->
      GetHardwareInterface("base")); //TODO Magic string
  if(hardwareInterface)
    hardwareInterface->EnqueueCommand({bestControl}, _dt);
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
