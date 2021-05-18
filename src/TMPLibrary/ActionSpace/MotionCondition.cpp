#include "MotionCondition.h"

#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupRoadmap.h"

#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

/*----------------------- Construction -----------------------*/

MotionCondition::
MotionCondition() {
  this->SetName("MotionCondition");
}

MotionCondition::
MotionCondition(XMLNode& _node) : Condition(_node) {
  this->SetName("MotionCondition");

  std::string robotGroupLabel = _node.Read("robotGroup", true, "", 
                                      "Robot to assign constrant.");

  auto robotGroup = this->GetMPProblem()->GetRobotGroup(robotGroupLabel);

  for(auto& child : _node) {
    if(child.Name() == "Constraint") {
      auto robotLabel = child.Read("robot", false, "", "Robot to assign constraint.");
      auto robot = (robotLabel != "") ? robotGroup->GetRobot(robotLabel)
                                      : nullptr;
      auto constraint = Constraint::Factory(robot, child);
      m_constraints.push_back(std::move(constraint));
    }
  }
}

MotionCondition::
~MotionCondition() {}

/*------------------------ Interface -------------------------*/ 

RobotGroup*
MotionCondition::
Satisfied(const State& _state) const {

  // This implementation assumes there is only one valid
  // pairing of robot to constraint. Scenarios where one
  // robot satisfies two constraints (a,b) and another 
  // robot only satisfied one constraint (a) can lead to missed
  // satisfying states.

  // This can be fixed with a more robust implementation, but 
  // it should be fine in the meantime while we use point-based
  // CSpace constraints.

  for(auto& kv : _state) {
    std::set<Robot*> matched;

    // Get the robot group and group cfg
    auto group = kv.first;
    auto roadmap = kv.second.first;
    auto vid = kv.second.second;
    auto groupCfg = roadmap->GetVertex(vid);

    bool foundMatch = true;
    for(auto& constraint : m_constraints) {

      foundMatch = false;

      for(auto& robot : group->GetRobots()) {

        // Make sure robot has not already been used.
        if(matched.count(robot))
          continue;

        // Make sure the robot is of the right type
        if(constraint->GetRobot() and 
           constraint->GetRobot()->GetCapability() != robot->GetCapability())
          continue;

        // Check if robot cfg satisfies constraint
        auto cfg = groupCfg.GetRobotCfg(robot);
        if(!constraint->Satisfied(cfg))
          continue;

        constraint->SetRobot(robot);
        matched.insert(robot);
        foundMatch = true;
      }

      if(!foundMatch) {
        break;
      }
    }

    // If all constraints are satisfied, return the robot group
    if(foundMatch) {
      return group;
    }
  }

  return nullptr;
}

/*------------------------------------------------------------*/
