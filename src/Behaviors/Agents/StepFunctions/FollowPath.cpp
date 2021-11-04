#include "FollowPath.h"

#include "Behaviors/Agents/PathFollowingAgent.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Constraints/BoundaryConstraint.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "Simulator/BulletModel.h"
#include "Simulator/Simulation.h"
#include "Utilities/XMLNode.h"

/*------------------------------ Construction ------------------------------*/

FollowPath::
FollowPath(Agent* _agent, XMLNode& _node) : StepFunction(_agent, _node) {
  m_waypointDm = _node.Read("waypointDm", true, "",
      "The distance metric to use for checking proximity to a path waypoint.");

  m_waypointThreshold = _node.Read("waypointThreshold", false,
      m_waypointThreshold, 0., std::numeric_limits<double>::max(),
      "The robot is considered to have reached the waypoints within this threshold.");
}

FollowPath::
~FollowPath() {}

/*-------------------------------- Interface -------------------------------*/

void
FollowPath::
StepAgent(double _dt) {
  auto p = dynamic_cast<PathFollowingAgent*>(m_agent);
  if(p and p->HasPlan())
    ExecutePath(_dt);
}

/*----------------------------- Helper Functions ----------------------------*/

void
FollowPath::
ExecutePath(double _dt) {
  auto p = dynamic_cast<PathFollowingAgent*>(m_agent);
  auto path = p->GetPath();
  std::cout << "Current: " << path[m_pathIndex] << std::endl;

  // Get the current waypoint
  auto current = path[m_pathIndex];

  // Check if the robot has reached the current waypoint
  std::cout << "Reached: " << ReachedWaypoint(current) << std::endl;
  if(!ReachedWaypoint(current)) {
    MoveToWaypoint(current, _dt);
    return;
  }

  // Iterate the index to the next waypoint
  std::cout << "Path Index: " << m_pathIndex << std::endl;
  m_pathIndex++;

  // Check if the robot has completed the path
  if(m_pathIndex == path.size()) {
    // Clear the old plan and reset the index
    p->ClearPlan();
    m_pathIndex = 0;
    return;
  }

  // Move to the next waypoint
  auto next = path[m_pathIndex];
  MoveToWaypoint(next, _dt);
}

bool
FollowPath::
ReachedWaypoint(const Cfg& _waypoint) {
  auto p = dynamic_cast<PathFollowingAgent*>(m_agent);
  auto lib = p->GetMPLibrary();
  auto dm = lib->GetDistanceMetric(m_waypointDm);

  Cfg state = p->GetRobot()->GetSimulationModel()->GetState();

  double distance = dm->Distance(state, _waypoint);

  return distance < m_waypointThreshold;
}

void
FollowPath::
MoveToWaypoint(const Cfg& _waypoint, double _dt) {

  const size_t steps = Simulation::NearestNumSteps(_dt);;
  auto p = dynamic_cast<PathFollowingAgent*>(m_agent);
  auto robot = p->GetRobot();
  const double timeRes = robot->GetMPProblem()->GetEnvironment()->GetTimeRes(),
               time = timeRes * steps;

  // Ask the controller for the best action to get from the current position to
  // the next waypoint.
  const Cfg current = robot->GetSimulationModel()->GetState();
  auto bestControl = robot->GetController()->operator()(current, _waypoint, time);


  if(m_debug){
    std::cout << "Printing m_path in agent step function" << std::endl;
    auto path = p->GetPath();
    for(auto cfg : path){
      std::cout << cfg.PrettyPrint() << std::endl;
    }
  }

  //TODO::Add hardware version
  robot->GetSimulationModel()->Execute(bestControl);
}
