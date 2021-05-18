#include "Interaction.h"

#include "TMPLibrary/InteractionStrategies/InteractionStrategyMethod.h"

/*----------------------- Construction -----------------------*/

Interaction::
Interaction() {
  this->SetName("Interaction");
}

Interaction::
Interaction(XMLNode& _node) : Action(_node) {
  this->SetName("Interaction");
  ParseXMLNode(_node);
}

Interaction::
~Interaction() {}

/*------------------------ Interface -------------------------*/

bool
Interaction::
Valid(const State& _state) {

  // Check if pre-conditions are satisfied.
  if(!Action::Valid(_state))
    return false;

  // Check if there is a valid motion from pre-conditions
  // to post-conditions.
  auto is = this->GetInteractionStrategyMethod(m_isLabel);
  if(!is->operator()(this,_state))
    return false;

  return true;
}

/*------------------------ Accessors -------------------------*/

const std::vector<std::unique_ptr<Condition>>&
Interaction::
GetInterimConditions() const {
  return m_interimConditions;
}

Interaction::MPSolution*
Interaction::
GetMPSolution() const {
  return m_mpSolution.get();
}

/*--------------------- Helper Functions ---------------------*/

void
Interaction::
SetClassName() {
  this->SetName("Interaction");
}

void
Interaction::
ParseXMLNode(XMLNode& _node) {
  Action::ParseXMLNode(_node);

  m_isLabel = _node.Read("isLabel", true, "", 
                         "Interaction Strategy Label");

  for(auto& child : _node) {
    if(_node.Name() == "InterimConditions") {
      for(auto& grandchild : child) {
        auto condition = Condition::Factory(grandchild);
        m_interimConditions.push_back(std::move(condition));
      }
    }
  }
}

/*------------------------------------------------------------*/
