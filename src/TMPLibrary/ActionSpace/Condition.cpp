#include "Condition.h"

#include "FormationCondition.h"
#include "MotionCondition.h"

#include "MPProblem/RobotGroup/RobotGroup.h"
/*----------------------- Construction -----------------------*/

Condition::
Condition() {}

Condition::
Condition(XMLNode& _node) : TMPBaseObject(_node) {}

Condition::
~Condition() {}

std::unique_ptr<Condition>
Condition::
Factory(XMLNode& _node) {

  
  std::string type = _node.Read("type", true, "", "The Condition class name.");
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  std::unique_ptr<Condition> output;

  if(type == "formation") {
    output = std::unique_ptr<FormationCondition>(
                new FormationCondition(_node));
  }
  else if(type == "motion") {
    output = std::unique_ptr<MotionCondition>(
                new MotionCondition(_node));
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
