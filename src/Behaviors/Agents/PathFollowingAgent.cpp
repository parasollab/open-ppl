#include "PathFollowingAgent.h"

#include <limits>

#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/HardwareInterfaces/QueuedHardwareInterface.h"


/*------------------------------ Construction --------------------------------*/

PathFollowingAgent::
PathFollowingAgent(Robot* const _r) : PlanningAgent(_r) { }


PathFollowingAgent::
PathFollowingAgent(Robot* const _r, const PathFollowingAgent& _a)
  : PlanningAgent(_r, _a)
{ }


PathFollowingAgent::
PathFollowingAgent(Robot* const _r, XMLNode& _node) : PlanningAgent(_r) {
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
Uninitialize() {
  if(!m_initialized)
    return;
  PlanningAgent::Uninitialize();

  ClearPlan();
}

/*--------------------------------- Planning ---------------------------------*/

bool
PathFollowingAgent::
HasPlan() const {
  return !m_path.empty();
}


void
PathFollowingAgent::
ClearPlan() {
  PlanningAgent::ClearPlan();
  m_path.clear();
  m_pathIndex = 0;
}


/*----------------------------- Planning Helpers -----------------------------*/

void
PathFollowingAgent::
WorkFunction(std::shared_ptr<MPProblem> _problem) {
  PlanningAgent::WorkFunction(_problem);

  m_path = m_solution->GetPath()->Cfgs();
  m_pathIndex = 0;

  // Throw if PMPL failed to generate a solution.
  if(m_path.empty())
    throw RunTimeException(WHERE, "PMPL failed to produce a solution.");

}

/*------------------------------- Task Helpers -------------------------------*/

bool
PathFollowingAgent::
EvaluateTask() {
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

  if(GetTask()->EvaluateGoalConstraints({current})) {
    if(true)
      std::cout << "Reached the end of the path." << std::endl;
    GetTask()->SetCompleted();
    SetTask(nullptr);
    return false;
  }
  // Advance our subgoal while we are within the distance threshold of the next
  // one.
  while(distance < threshold) {
    if(m_debug)
      std::cout << "\tReached waypoint " << m_pathIndex << " at "
                << distance << "/" << threshold
                << "\n\t\t" << m_path[m_pathIndex].PrettyPrint()
                << std::endl;

    // Move to next cfg in path since the distance is within the threshold.
    ++m_pathIndex;

    // Check if we have completed the path. If so, this task is complete.
    if(m_pathIndex == m_path.size() or GetTask()->EvaluateGoalConstraints({current}))
    {
      if(true)
        std::cout << "Reached the end of the path." << std::endl;
      GetTask()->SetCompleted();
      SetTask(nullptr);
      return false;
    }

    distance = dm->Distance(current, m_path[m_pathIndex]);
  }

  return true;
}


void
PathFollowingAgent::
ExecuteTask(const double _dt) {
  if(m_debug)
    std::cout << "Approaching waypoint " << m_pathIndex << " / "
              << m_path.size() - 1 << "."
              << std::endl;

  // Get the smallest safe time interval in case _dt is too fast for the hardware.
  const size_t steps = std::max(NearestNumSteps(_dt), MinimumSteps());
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes(),
               time = timeRes * steps;

  const Cfg current = m_robot->GetDynamicsModel()->GetSimulatedState();
  auto bestControl = m_robot->GetController()->operator()(current,
      m_path[m_pathIndex], time);
  ExecuteControls({bestControl}, steps);
}

/*----------------------------------------------------------------------------*/
