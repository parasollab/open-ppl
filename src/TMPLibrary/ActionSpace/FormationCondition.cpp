#include "FormationCondition.h"

#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

/*----------------------- Construction -----------------------*/

FormationCondition::
FormationCondition() {
  this->SetName("FormationCondition");
}

FormationCondition::
FormationCondition(XMLNode& _node) : Condition(_node) {
  this->SetName("FormationCondition");

  for(auto& child : _node) {
    if(child.Name() == "Member") {
      auto type = child.Read("type", true, "", 
                        "Type of robot required in formation.");
      m_requiredTypes.push_back(type);
    }
  }

  this->m_unique = true;
}

FormationCondition::
~FormationCondition() {}
/*------------------------ Interface -------------------------*/

RobotGroup*
FormationCondition::
Satisfied(const State& _state) const {

  for(auto& kv : _state) {
    auto robotGroup = kv.first;
    if(CheckRequirements(robotGroup))
      return robotGroup;
  }

  return nullptr;
}

/*--------------------- Helper Functions ---------------------*/

bool
FormationCondition::
CheckRequirements(RobotGroup* _group) const {
  std::set<Robot*> used;

  // Check if the robot group meets all of the constraints
  for(const auto& type : m_requiredTypes) {

    bool match = false;

    for(auto& robot : _group->GetRobots()) {
      // Check if this robot has been used already
      if(used.count(robot)) 
        continue;

      if(robot->GetCapability() == type) {
        match = true;
        used.insert(robot);
        break;
      }
    }

    if(!match) {
      return false;
    }
  }

  return true;
}

/*------------------------------------------------------------*/
