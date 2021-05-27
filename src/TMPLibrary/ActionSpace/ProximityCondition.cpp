#include "ProximityCondition.h"

#include "MPProblem/RobotGroup/RobotGroup.h"

#include "TMPLibrary/TMPLibrary.h"

/*----------------------- Construction -----------------------*/

ProximityCondition::
ProximityCondition() {
  this->SetName("ProximityCondition");
}

ProximityCondition::
ProximityCondition(XMLNode& _node, TMPLibrary* _library) 
                  : Condition(_node,_library) {
  ParseXML(_node);
}

ProximityCondition::
~ProximityCondition() {}
/*------------------------ Interface -------------------------*/

RobotGroup*
ProximityCondition::
Satisfied(const State& _state) const {
  return nullptr;
}

/*--------------------- Helper Functions ---------------------*/ 

void
ProximityCondition::
ParseXML(XMLNode& _node) {
  this->SetName("ProximityCondition");
}

/*------------------------------------------------------------*/
