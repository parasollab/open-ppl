#include "CompositeCondition.h"


/*----------------------- Construction -----------------------*/

CompositeCondition::
CompositeCondition() {
  this->SetName("CompositeCondition");
}

CompositeCondition::
CompositeCondition(XMLNode& _node) : Condition(_node) {
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

GroupRobot*
CompositeCondition::
Satisfied(const State& _state) {
  return nullptr;
}
/*------------------------ Accessors -------------------------*/

bool
CompositeCondition::
IsUnique() const {
  //TODO::Return OR of all subcondition->IsUnique()
  return true;
}

/*------------------------------------------------------------*/
