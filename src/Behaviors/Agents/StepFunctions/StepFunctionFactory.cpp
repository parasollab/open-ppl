#include "StepFunction.h"
#include "PlanGroupPathStepFunction.h"
#include "EmptyStepFunction.h"
#include "FollowPath.h"

#include <algorithm>
#include <string>

std::unique_ptr<StepFunction>
StepFunction::
Factory(Agent* _agent, XMLNode& _node) {
  // Read the node and mark it as visited.
  std::string type = _node.Read("type", true, "", "The Agent class name.");
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  std::unique_ptr<StepFunction> output;

  if(type == "plangrouppath") {
    Coordinator* c = static_cast<Coordinator*>(_agent);
    if(c) {
      output = std::unique_ptr<StepFunction>(
          new PlanGroupPathStepFunction(c, _node)
      );
    }
  }
  // else if(type == "followpath") {
  //   output = std::unique_ptr<StepFunction>(
  //     new FollowPath(_agent, _node)
  //   );
  // }
  else if(type == "empty") {
    output = std::unique_ptr<StepFunction>(
      new EmptyStepFunction(_agent,_node)
    );
  }
  else {
    throw ParseException(_node.Where(), "Unknown step function type '" + type + "'.");
  }

  // Read the debug flag.
  output->m_debug = _node.Read("debug", false, false, "Show debug messages.");

  return output;
}
