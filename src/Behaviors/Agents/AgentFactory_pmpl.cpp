#include "Agent.h"

#include "Utilities/XMLNode.h"


std::unique_ptr<Agent>
Agent::
Factory(Robot* const _r, XMLNode& _node) {
  // If we are not building the simulator, ignore the agent node.
  _node.Ignore();

  return {nullptr};
}
