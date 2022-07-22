#include "ScheduledCBS.h"


/*----------------------- Construction -----------------------*/

ScheduledCBS::
ScheduledCBS() {
  this->SetName("ScheduledCBS");
}

ScheduledCBS::
ScheduledCBS(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("ScheduledCBS");
}

ScheduledCBS::
~ScheduledCBS() {

}

/*------------------------ Overrides -------------------------*/
void
ScheduledCBS::
Initialize() {

}

bool
ScheduledCBS::
Run(Plan* _plan) {
  if(!_plan)
    _plan = this->GetPlan();

  return false;
}
/*----------------------- CBS Functors -----------------------*/

bool
ScheduledCBS::
LowLevelPlanner(Node& _node, SemanticTask* _task) {
  return false;
}

std::vector<std::pair<SemanticTask*,ScheduledCBS::Constraint>>
ScheduledCBS::
ValidationFunction(Node& _node) {
  return {};
}

double
ScheduledCBS::
CostFunction(Node& _node) {
  return 0.0;
}

void
ScheduledCBS::
InitialSolutionFunction(std::vector<Node>& _root, std::vector<SemanticTask*> _tasks,
                        CBSLowLevelPlanner<SemanticTask,Constraint,GroupPathType>& _lowLevel,
                        CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost) {
                          
}

/*--------------------- Helper Functions ---------------------*/

ScheduledCBS::GroupPathType*
ScheduledCBS::
QueryPath(SemanticTask* _task, const double _startTime,
                         const Node& _node) {
  return nullptr;
}

void
ScheduledCBS::
ConvertToPlan(const Node& _node) {

}

/*------------------------------------------------------------*/