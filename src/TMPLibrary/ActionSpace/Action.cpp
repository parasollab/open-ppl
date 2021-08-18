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

  // Check if each of the initial stage conditions are satisfied.
  const auto& stage = m_stages[0];
  const auto& conditions = m_stageConditions[stage];
  for(const auto& label : conditions) {
    auto pre = this->GetTMPLibrary()->GetActionSpace()->GetCondition(label);
    if(!pre->Satisfied(_state))
    return false;
  }
  return true;
}

/*------------------------ Accessors -------------------------*/

const std::vector<std::string>&
Action::
GetStages() const {
  return m_stages;
}

const std::vector<std::string>&
Action::
GetStageConditions(const std::string& _stage) const {
  return m_stageConditions.at(_stage);
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
    if(child.Name() == "Stage") {
      std::string label = child.Read("label",true,"",
                                     "The label for this stage.");
      m_stages.push_back(label);

      for(auto& grandchild : child) {
        auto condition = grandchild.Read("label",true,"",
                         "Label of condition to include.");
        m_stageConditions[label].push_back(condition);
      }
    }
  }
}

/*------------------------------------------------------------*/
