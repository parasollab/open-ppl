#include "FollowPath.h"

#include "Behaviors/Agents/PathFollowingAgent.h"
#include "ConfigurationSpace/Cfg.h"
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
    ExecutePath();
}

/*----------------------------- Helper Functions ----------------------------*/

void
FollowPath::
ExecutePath() {
  auto p = dynamic_cast<PathFollowingAgent*>(m_agent);
  auto path = p->GetPath();

  // Get the current waypoint
  auto current = path[m_pathIndex];
  
  // Check if the robot has reached the current waypoint
  if(!ReachedWaypoint(current)) {
    MoveToWaypoint(current);
    return;
  }

  // Iterate the index to the next waypoint
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
  MoveToWaypoint(next);
}
    
bool 
FollowPath::
ReachedWaypoint(const Cfg& _waypoint) {
  return true;
}

void
FollowPath::
MoveToWaypoint(const Cfg& _cfg) {

}
