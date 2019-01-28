#include "State.h"
#include "Actions/Action.h"

State::
State(){
}

State::
~State() = default;

bool
State::
operator==(const State& _state) const{
 if(m_robotLocations != _state.m_robotLocations or
     m_objectLocations != _state.m_objectLocations or
     m_taskOwners != _state.m_taskOwners)
   return false;
 return true;
}

bool
State::
operator!=(const State& _state) const{
 return !(*this == _state);
}

State
State::
ApplyAction(std::shared_ptr<Action> _action){
  State result = *this;
  State changes = _action->GetResultState();

  if(changes.m_objectLocations.size() != 0){
    result.m_objectLocations = changes.m_objectLocations;
  }

  for(auto robotLocation : changes.m_robotLocations){
    auto robot = robotLocation.first;
    auto location = robotLocation.second;
    result.m_robotLocations[robot] = location;
  }

  if(changes.m_taskOwners.size() != 0){
    result.m_taskOwners = changes.m_taskOwners;
  }

  return result;
}
