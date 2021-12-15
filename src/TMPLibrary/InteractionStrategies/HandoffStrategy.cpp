#include "HandoffStrategy.h"

#include "TMPLibrary/ActionSpace/Interaction.h"

/*----------------------- Construction -----------------------*/

HandoffStrategy::
HandoffStrategy() {
  this->SetName("HandoffStrategy");
}

HandoffStrategy::
HandoffStrategy(XMLNode& _node) : GraspStrategy(_node) {
  this->SetName("HandoffStrategy");
}

HandoffStrategy::
~HandoffStrategy() {}

/*------------------------ Interface -------------------------*/

bool
HandoffStrategy::
operator()(Interaction* _interaction, State& _start) {

  _interaction->Initialize();
  auto stages = _interaction->GetStages();

  // Assign initial roles
  auto initialConditions = _interaction->GetStageConditions(stages[0]);
  AssignRoles(_start,initialConditions);

  return false;
}

/*--------------------- Helper Functions ---------------------*/

HandoffStrategy::State
HandoffStrategy::
GenerateTransitionState(Interaction* _interaction, const State& _previous, const size_t _next) {
  return State();
}
/*------------------------------------------------------------*/
