#include "Agent.h"

#include "BatteryConstrainedGroup.h"
#include "CentralPlanner.h"
#include "Coordinator.h"
#include "DummyAgent.h"
#include "HandoffAgent.h"
#include "PathFollowingChildAgent.h"
#include "PathFollowingAgent.h"
#include "PlanningAgent.h"
#include "RoadmapFollowingAgent.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include <algorithm>
#include <string>


std::unique_ptr<Agent>
Agent::
Factory(Robot* const _r, XMLNode& _node) {
  // Read the node and mark it as visited.
  std::string type = _node.Read("type", true, "", "The Agent class name.");
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  std::unique_ptr<Agent> output;

  if(type == "pathfollowing")
    output = std::unique_ptr<PathFollowingAgent>(
        new PathFollowingAgent(_r, _node)
    );
  else if(type == "planning")
    output = std::unique_ptr<PlanningAgent>(
        new PlanningAgent(_r, _node)
    );
  else if(type == "pathfollowingchild")
    output = std::unique_ptr<PathFollowingChildAgent>(
        new PathFollowingChildAgent(_r, _node)
    );
  else if(type == "roadmapfollowing")
    output = std::unique_ptr<RoadmapFollowingAgent>(
        new RoadmapFollowingAgent(_r, _node)
    );
  else if(type == "batteryconstrainedgroup")
    output = std::unique_ptr<BatteryConstrainedGroup>(
        new BatteryConstrainedGroup(_r, _node)
    );
  else if(type == "handoff")
    output = std::unique_ptr<HandoffAgent>(
        new HandoffAgent(_r, _node)
    );
  else if(type == "coordinator")
    output = std::unique_ptr<Coordinator>(
        new Coordinator(_r, _node)
    );
  else if(type == "centralplanner")
    output = std::unique_ptr<CentralPlanner>(
        new CentralPlanner(_r, _node)
    );
  else if(type == "dummy")
    output = std::unique_ptr<DummyAgent>(
        new DummyAgent(_r, _node)
    );
  else
    throw ParseException(_node.Where(), "Unknown agent type '" + type + "'.");

  // Read the debug flag.
  output->m_debug = _node.Read("debug", false, false, "Show debug messages.");

  return output;
}
