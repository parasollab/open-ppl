#include "Condition.h"

#include "CompositeCondition.h"
#include "FormationCondition.h"
#include "MotionCondition.h"
#include "ProximityCondition.h"

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

  if(type == "composite") {
    output = new CompositeCondition(_node,_tmpLibrary);
  }
  else if(type == "formation") {
    output = new FormationCondition(_node,_tmpLibrary);
  }
  else if(type == "motion") {
    output = new MotionCondition(_node,_tmpLibrary);
  }
  else if(type == "proximity") {
    output = new ProximityCondition(_node,_tmpLibrary);
  }

  return output;
}

/*------------------------ Interface -------------------------*/

RobotGroup*
Condition::
Satisfied(const State& _state) const {
  return nullptr;
}

void 
Condition::
AssignRoles(std::unordered_map<std::string,Robot*>& _roleMap,
            const State& _state) const { }

/*------------------------ Accessors -------------------------*/

bool
Condition::
IsUnique() const {
  return m_unique;
}

/*------------------------------------------------------------*/
