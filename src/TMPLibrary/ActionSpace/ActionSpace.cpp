#include "ActionSpace.h"

#include "Action.h"
#include "Condition.h"
#include "Interaction.h"

#include "TMPLibrary/TMPLibrary.h"

#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

/*----------------------- Construction -----------------------*/

ActionSpace::
ActionSpace(TMPLibrary* _tmpLibrary) : m_tmpLibrary(_tmpLibrary) {}

ActionSpace::
~ActionSpace() {
  for(auto& pair : m_conditions)
    delete pair.second;
  for(auto& pair : m_actions)
    delete pair.second;
}

/*------------------------- Interface ------------------------*/

void
ActionSpace::
Initialize() {
  for(auto action : m_actions) {
    action.second->Initialize();
  }
}
/*------------------------ Conditions ------------------------*/

Condition*
ActionSpace::
GetCondition(const std::string& _label) {
  return GetUtility(_label, m_conditions);
}

void
ActionSpace::
SetCondition(const std::string& _label, Condition* _utility) {
  SetUtility(_label, _utility, m_conditions);
}

/*-------------------------- Actions -------------------------*/

const ActionSpace::LabelMap<Action>&
ActionSpace::
GetActions() {
  return m_actions;
}

Action*
ActionSpace::
GetAction(const std::string& _label) {
  return GetUtility(_label, m_actions);
}

void
ActionSpace::
SetAction(const std::string& _label, Action* _utility) {
  SetUtility(_label, _utility, m_actions);
}

/*--------------------- Helper Functions ---------------------*/

void
ActionSpace::ParseXML(XMLNode& _node) {
  for(auto& child : _node) {
    if(child.Name() == "Condition") {
      auto utility = Condition::Factory(child,m_tmpLibrary);
      if(m_conditions.count(utility->GetLabel())) {
        throw ParseException(WHERE) << "Second condition with the label '"
                                    << utility->GetLabel()
                                    << "'. Labels must be unique.";
      }
      SetCondition(utility->GetLabel(), utility);
    }
    else if(child.Name() == "Action") {
      auto utility = new Action(child);
      if(m_actions.count(utility->GetLabel())) {
        throw ParseException(WHERE) << "Second condition with the label '"
                                    << utility->GetLabel()
                                    << "'. Labels must be unique.";
      }
      SetAction(utility->GetLabel(), utility);
    }
    else if(child.Name() == "Interaction") {
      auto utility = new Interaction(child);
      if(m_actions.count(utility->GetLabel())) {
        throw ParseException(WHERE) << "Second condition with the label '"
                                    << utility->GetLabel()
                                    << "'. Labels must be unique.";
      }
      SetAction(utility->GetLabel(), utility);
    }
  }
}

template <typename Utility>
Utility*
ActionSpace::
GetUtility(const std::string& _label, const LabelMap<Utility>& _map) {
  try {
    return _map.at(_label);
  }
  catch(const std::out_of_range&) {
    Utility dummy;
    throw RunTimeException(WHERE) << "Error when fetching" 
                                  << dummy.GetName()
                                  << " '"
                                  << _label
                                  << "' does not exists.";
  }
  catch(const std::exception& _e) {
    Utility dummy;
    throw RunTimeException(WHERE) << "Error when fetching" 
                                  << dummy.GetName()
                                  << " '"
                                  << _label
                                  << "' : "
                                  << _e.what();
  }
  catch(...) {
    Utility dummy;
    throw RunTimeException(WHERE) << "Error when fetching" 
                                  << dummy.GetName()
                                  << " '"
                                  << _label
                                  << "' : (unknown).";
  }
  return nullptr;
}

template <typename Utility>
void
ActionSpace::
SetUtility(const std::string& _label, Utility* _utility,
           LabelMap<Utility>& _map) {
  // Set the library pointer.
  _utility->SetTMPLibrary(m_tmpLibrary);

  // Check if this label is already in use.
  auto iter = _map.find(_label);
  const bool alreadyExists = iter != _map.end();

  // If the label already exists, we need to release the previous utility first.
  if(alreadyExists) {
    delete iter->second;
    iter->second = _utility;
  }
  else {
    _map.insert({_label, _utility});
  }
}

/*------------------------------------------------------------*/
