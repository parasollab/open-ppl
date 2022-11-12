#include "SimpleTaskAllocationMethod.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/TaskAllocators/TaskAllocatorMethod.h"

/*------------------------- Construction ------------------------*/

SimpleTaskAllocationMethod::
SimpleTaskAllocationMethod() {
  this->SetName("SimpleTaskAllocationMethod");
}

SimpleTaskAllocationMethod::
SimpleTaskAllocationMethod(XMLNode& _node) : TMPStrategyMethod(_node) {
  this->SetName("SimpleTaskAllocationMethod");
}

SimpleTaskAllocationMethod::
~SimpleTaskAllocationMethod() {}

/*****************************************Call Method****************************************************/

void
SimpleTaskAllocationMethod::
AssignTasks() {
  this->GetTaskAllocator(m_teLabel)->AllocateTasks();
}
