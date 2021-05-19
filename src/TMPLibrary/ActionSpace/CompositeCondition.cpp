#include "CompositeCondition.h"

#include "MPProblem/RobotGroup/RobotGroup.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/TMPLibrary.h"

/*----------------------- Construction -----------------------*/

CompositeCondition::
CompositeCondition() {
  this->SetName("CompositeCondition");
}

CompositeCondition::
CompositeCondition(XMLNode& _node, TMPLibrary* _tmpLibrary) : Condition(_node,_tmpLibrary) {
  this->SetName("CompositeCondition");
  
  for(auto child : _node) {
    if(child.Name() == "Subcondition") {
      std::string label = child.Read("label",true,"",
                                     "Label of subcondition.");
      m_subconditions.push_back(label);
    }
  }
}

CompositeCondition::
~CompositeCondition() {}
/*------------------------ Interface -------------------------*/

RobotGroup*
CompositeCondition::
Satisfied(const State& _state) {

  auto actionSpace = this->GetTMPLibrary()->GetActionSpace();
  State refinedState = _state;

  for(auto label : m_subconditions) {
    // Fetch the sub condition.
    auto sub = actionSpace->GetCondition(label);

    
    // Find the robot groups that satisfies the sub condtion.
    std::vector<RobotGroup*> toErase;

    for(auto kv : refinedState) {
      // Isolate the robot group and it's state
      State subState;
      subState[kv.first] = kv.second;
    
      // Check if the group satisfies the subcondition.
      auto group = sub->Satisfied(subState);

      // If it does not, remove it from the refined state.
      if(!group) {
        toErase.push_back(kv.first);
      }
    }
    
    // Remove unsatisfying robot groups from refined state.
    for(auto group : toErase) {
      refinedState.erase(group);
    }

    // If there is no satisfying group, return false;
    if(refinedState.empty())
      return nullptr;
  }

  return refinedState.begin()->first;
}
/*------------------------ Accessors -------------------------*/

bool
CompositeCondition::
IsUnique() const {
  //TODO::Return OR of all subcondition->IsUnique()
  return true;
}

/*------------------------------------------------------------*/
