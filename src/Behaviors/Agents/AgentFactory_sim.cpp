#include "Agent.h"

#include "BatteryConstrainedGroup.h"
#include "PathFollowingChildAgent.h"
#include "PathFollowingAgent.h"
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
  else if(type == "pathfollowingchildagent")
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
  else
    throw ParseException(_node.Where(), "Unknown agent type '" + type + "'.");

  // Read the debug flag.
  output->m_debug = _node.Read("debug", false, false, "Show debug messages.");

  return output;
}
