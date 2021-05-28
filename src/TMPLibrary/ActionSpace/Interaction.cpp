#include "Interaction.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/InteractionStrategies/InteractionStrategyMethod.h"
#include "TMPLibrary/Solution/Plan.h"

/*----------------------- Construction -----------------------*/

Interaction::
Interaction() {
  this->SetName("Interaction");
}

Interaction::
Interaction(XMLNode& _node) : Action(_node) {
  ParseXMLNode(_node);
}

Interaction::
~Interaction() {}

/*------------------------ Interface -------------------------*/

void
Interaction::
Initialize() {
  auto c = this->GetPlan()->GetCoordinator();
  m_toInterimSolution = std::unique_ptr<MPSolution>(new MPSolution(c->GetRobot()));
  m_toPostSolution = std::unique_ptr<MPSolution>(new MPSolution(c->GetRobot()));
}

bool
Interaction::
Valid(const State& _state) {

  // Check if pre-conditions are satisfied.
  if(!Action::Valid(_state))
    return false;

  // TODO:: This is being done elsewhere - re-evaluate the decision.
  // Check if there is a valid motion from pre-conditions
  // to post-conditions.
  /*State s = _state;
  auto is = this->GetInteractionStrategyMethod(m_isLabel);
  if(!is->operator()(this,s))
    return false;
  */
  return true;
}

/*------------------------ Accessors -------------------------*/

const std::vector<std::string>&
Interaction::
GetInterimConditions() const {
  return m_interimConditions;
}

Interaction::MPSolution*
Interaction::
GetToInterimSolution() const {
  return m_toInterimSolution.get();
}
    
Interaction::MPSolution*
Interaction::
GetToPostSolution() const {
  return m_toPostSolution.get();
}
    
const std::string 
Interaction::
GetInteractionStrategyLabel() const {
  return m_isLabel;
}
    
void 
Interaction::
SetToInterimPath(GroupPathType* _path) {
  m_toInterimPath = _path;
}

Interaction::GroupPathType*
Interaction::
GetToInterimPath() {
  return m_toInterimPath;
}

void
Interaction::
SetToPostPath(GroupPathType* _path) {
  m_toPostPath = _path;
}

Interaction::GroupPathType*
Interaction::
GetToPostPath() {
  return m_toPostPath;
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
  //Action::ParseXMLNode(_node);

  m_isLabel = _node.Read("isLabel", true, "", 
                         "Interaction Strategy Label");

  for(auto& child : _node) {
    if(child.Name() == "InterimConditions") {
      for(auto& grandchild : child) {
        auto condition = grandchild.Read("label",true,"",
                         "Label of condition to include.");
        m_interimConditions.push_back(std::move(condition));
      }
    }
  }
}

/*------------------------------------------------------------*/
