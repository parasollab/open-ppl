#include "Action.h"

#include "ActionSpace.h"

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
  for(const auto& label : m_preConditions) {
    auto pre = this->GetTMPLibrary()->GetActionSpace()->GetCondition(label);
    if(!pre->Satisfied(_state))
    return false;
  }
  return true;
}

/*------------------------ Accessors -------------------------*/

const std::vector<std::string>&
Action::
GetPreConditions() const {
  return m_preConditions;
}

const std::vector<std::string>&
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
    if(child.Name() == "PreConditions") {
      for(auto& grandchild : child) {
        auto condition = grandchild.Read("label",true,"",
                         "Label of condition to include.");
        m_preConditions.push_back(std::move(condition));
      }
    }
    else if(child.Name() == "PostConditions") {
      for(auto& grandchild : child) {
        auto condition = grandchild.Read("label",true,"",
                         "Label of condition to include.");
        m_postConditions.push_back(std::move(condition));
      }
    }
  }
}

/*------------------------------------------------------------*/
