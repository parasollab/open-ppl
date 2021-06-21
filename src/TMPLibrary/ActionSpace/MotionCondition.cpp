#include "MotionCondition.h"

#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupRoadmap.h"

#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include "TMPLibrary/TMPLibrary.h"

/*----------------------- Construction -----------------------*/

MotionCondition::
MotionCondition() {
  this->SetName("MotionCondition");
}

MotionCondition::
MotionCondition(XMLNode& _node, TMPLibrary* _tmpLibrary) : Condition(_node,_tmpLibrary) {
  this->SetName("MotionCondition");

  for(auto& child : _node) {
    auto found = child.Name().find("Constraint");
    if(found != std::string::npos) {

      // Generate constraint with specified type
      std::string type = child.Read("type", false, "ANY", 
                                    "Type of robot to apply the constraint to.");
      auto constraint = std::make_pair(type,Constraint::Factory(nullptr, child));
      m_constraints.push_back(std::move(constraint));

      // Set role for constraint
      std::string role = child.Read("role", true, "", 
                                    "Used to identify robot tole across conditions.");
      m_roles[m_constraints.back().second.get()] = role;
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
        if(constraint.first != robot->GetCapability())
          continue;

        // Check if robot cfg satisfies constraint
        auto cfg = groupCfg.GetRobotCfg(robot);
        constraint.second->SetRobot(robot);

        if(!constraint.second->Satisfied(cfg))
          continue;

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

/*------------------------- Accessors ------------------------*/

const std::vector<std::pair<std::string,std::unique_ptr<Constraint>>>&
MotionCondition::
GetConstraints() {
  return m_constraints;
}

std::vector<Constraint*>
MotionCondition::
GetConstraints(std::string _type) {
  std::vector<Constraint*> constraints;

  for(const auto& c : m_constraints) {
    if(_type == c.first) {
      constraints.push_back(c.second.get());
    }
  }

  return constraints;
}
    
std::string
MotionCondition::
GetRole(Constraint* _constraint) {
  return m_roles[_constraint];
}
/*------------------------------------------------------------*/
