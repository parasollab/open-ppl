#include "DummyStrategyMethod.h"

/*------------------------- Construction ------------------------*/

DummyStrategyMethod::
DummyStrategyMethod() {
  this->SetName("DummyStrategyMethod");
}

DummyStrategyMethod::
DummyStrategyMethod(XMLNode& _node) : TMPStrategyMethod(_node) {
  this->SetName("DummyStrategyMethod");
}

/*--------------------------- Overrides -------------------------*/
/*
void
DummyStrategyMethod::
PlanTasks() {}

void
DummyStrategyMethod::
AssignTasks() {}

void
DummyStrategyMethod::
DecomposeTasks() {}
*/
