#include "BasicTMPStrategyMethod.h"
#include "TMPLibrary/TaskAllocators/TaskAllocatorMethod.h"
#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"

/*-------------------------------- Constructor --------------------------------*/

BasicTMPStrategyMethod::
BasicTMPStrategyMethod() {
  this->SetName("BasicTMPStrategyMethod");
}

BasicTMPStrategyMethod::
BasicTMPStrategyMethod(XMLNode& _node) : TMPStrategyMethod(_node) {
  this->SetName("BasicTMPStrategyMethod");
}

BasicTMPStrategyMethod::
~BasicTMPStrategyMethod() { }

/*--------------------------- Helper Function Overrides -----------------------*/

void
BasicTMPStrategyMethod::
PlanTasks() {
  if(m_teLabel == "")
    return;

  auto te = this->GetTaskEvaluator(m_teLabel);
  te->operator()();
}

void
BasicTMPStrategyMethod::
DecomposeTasks() {
  //TODO::Uncomment when decomposer is fixed
  /*if(m_tdLabel == "")
    return;

  auto td = this->GetTaskEvaluator(m_teLabel);
  te->operarator()();*/
}

void
BasicTMPStrategyMethod::
AssignTasks() {
  if(m_taLabel == "")
    return;

  auto ta = this->GetTaskAllocator(m_taLabel);
  ta->AllocateTasks();
}

/*-----------------------------------------------------------------------------*/
