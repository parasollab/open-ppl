#include "Action.h"

/*----------------------- Construction -----------------------*/

Action::
Action() {}

Action::
Action(XMLNode& _node) : TMPBaseObject(_node) {
  SetClassName();
  ParseXMLNode(_node);
}

Action::
~Action() {}

/*------------------------ Interface -------------------------*/

bool
Action::
Valid(const State& _state) {

  // Check if each of the pre-conditions are satisfied.
  for(const auto& pre : m_preConditions) {
    if(!pre->Satisfied(_state))
    return false;
  }
  return true;
}

/*------------------------ Accessors -------------------------*/

const std::vector<std::unique_ptr<Condition>>&
Action::
GetPreConditions() const {
  return m_preConditions;
}

const std::vector<std::unique_ptr<Condition>>&
Action::
GetPostConditions() const {
  return m_postConditions;
}

/*--------------------- Helper Functions ---------------------*/

void
Action::
SetClassName() {
  this->SetName("Action");
}

void
Action::
ParseXMLNode(XMLNode& _node) {
  for(auto& child : _node) {
    if(_node.Name() == "PreConditions") {
      for(auto& grandchild : child) {
        auto condition = Condition::Factory(grandchild);
        m_preConditions.push_back(std::move(condition));
      }
    }
    else if(_node.Name() == "PostConditions") {
      for(auto& grandchild : child) {
        auto condition = Condition::Factory(grandchild);
        m_preConditions.push_back(std::move(condition));
      }
    }
  }
}

/*------------------------------------------------------------*/
