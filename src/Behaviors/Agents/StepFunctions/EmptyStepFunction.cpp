#include "EmptyStepFunction.h"

EmptyStepFunction::
EmptyStepFunction(Agent* _agent, XMLNode& _node) : StepFunction(_agent,_node) {}

EmptyStepFunction::
~EmptyStepFunction() {}

void
EmptyStepFunction::
StepAgent(double _dt) {}
