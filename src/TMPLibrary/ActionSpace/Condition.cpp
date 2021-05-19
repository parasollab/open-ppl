#include "Condition.h"

#include "FormationCondition.h"
#include "MotionCondition.h"

#include "MPProblem/RobotGroup/RobotGroup.h"

#include "TMPLibrary/TMPLibrary.h"

/*----------------------- Construction -----------------------*/

Condition::
Condition() {}

Condition::
Condition(XMLNode& _node, TMPLibrary* _tmpLibrary) : TMPBaseObject(_node) {
  this->SetTMPLibrary(_tmpLibrary);
}

Condition::
~Condition() {}

Condition*
Condition::
Factory(XMLNode& _node, TMPLibrary* _tmpLibrary) {

  
  std::string type = _node.Read("type", true, "", "The Condition class name.");
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  Condition* output;

  if(type == "formation") {
    output = new FormationCondition(_node,_tmpLibrary);
  }
  else if(type == "motion") {
    output = new MotionCondition(_node,_tmpLibrary);
  }

  return output;
}
/*------------------------ Interface -------------------------*/

RobotGroup*
Condition::
Satisfied(const State& _state) const {
  return nullptr;
}

/*------------------------ Accessors -------------------------*/

bool
Condition::
IsUnique() const {
  return m_unique;
}

/*------------------------------------------------------------*/
