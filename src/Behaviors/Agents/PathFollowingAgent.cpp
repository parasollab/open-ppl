#include "PathFollowingAgent.h"

#include <limits>

#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"


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

  /// @TODO Extract task properly. Currently we are using ye old query.
  MPTask* task = nullptr;

  // Initialize the agent's planning library.
  m_library = new MPLibrary(xmlFile);

  // Create a new solution object to hold a plan for this agent.
  auto solution = new MPSolution;

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

  // Get the current configuration.
  Cfg current(m_path[m_pathIndex].GetRobotIndex());
  current.SetData(m_robot->GetDynamicsModel()->GetSimulatedState());

  // Check if we've reached it. Advance the path index until the next subgoal is
  // at least .5 units away.
  auto dm = m_library->GetDistanceMetric("euclidean");
  while(dm->Distance(current, m_path[m_pathIndex]) < .5
      && m_pathIndex < m_path.size())
    ++m_pathIndex;

  // If we hit the end, return.
  if(m_pathIndex >= m_path.size())
    return;

  // Otherwise, execute the control that is nearest to the desired force.
  auto bestControl = m_robot->GetController()->operator()(current,
      m_path[m_pathIndex], _dt);
  bestControl.Execute();
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
