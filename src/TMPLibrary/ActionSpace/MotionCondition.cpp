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

  m_explicit = _node.Read("explicit",false,m_explicit,
                          "Flag indicating if constraints can be shifted.");

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

      m_roleConstraints[role] = m_constraints.back().second.get();
    }
  }

  ReCenter({0,0,0});
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

    const std::vector<std::pair<std::string,std::unique_ptr<Constraint>>>* constraints;
    if(m_explicit)
      constraints = &m_constraints;
    else
      constraints = &m_translatedConstraints;

    for(auto& constraint : *constraints) {

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
        break;
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

void
MotionCondition::
AssignRoles(std::unordered_map<std::string,Robot*>& _roleMap,
            const State& _state) const {
  std::unordered_set<Robot*> usedRobots;

  for(const auto kv : _state) {
    auto group = kv.first;
    auto robots = group->GetRobots();
    auto grm = kv.second.first;
    auto vid = kv.second.second;
    auto gcfg = grm->GetVertex(vid);

    const auto constraints = GetConstraints();
    for(const auto& c : constraints) {
      auto role = GetRole(c.second);

      // Find the right robot for the role.
      auto type = c.first;
      for(auto robot : robots) {
        // Check that the robot is of the right type.
        if(robot->GetCapability() != type)
          continue;

        // Check if the robot has been used already.
        if(usedRobots.count(robot))
          continue;

        // Check that the robot cfg matches the constraint.
        auto cfg = gcfg.GetRobotCfg(robot);
        if(!c.second->Satisfied(cfg))
          continue;

        _roleMap[role] = robot;
        usedRobots.insert(robot);
      }
    }
  }
}
/*------------------------- Accessors ------------------------*/

const std::vector<std::pair<std::string,Constraint*>>
MotionCondition::
GetConstraints() const {

  std::vector<std::pair<std::string,Constraint*>> constraints;

  if(m_explicit) {
    for(const auto& constraint : m_constraints) {
      auto p = std::make_pair(constraint.first,constraint.second.get());
      constraints.push_back(p);
    }
  }
  else {
    for(const auto& constraint : m_translatedConstraints) {
      auto p = std::make_pair(constraint.first,constraint.second.get());
      constraints.push_back(p);
    }
  }

  return constraints;
}

std::vector<Constraint*>
MotionCondition::
GetConstraints(std::string _type) const {
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
GetRole(Constraint* _constraint) const {
  return m_roles.at(_constraint);
}

std::set<std::string>
MotionCondition::
GetRoles() const {
  std::set<std::string> roles;

  for(auto kv : m_roles) {
    roles.insert(kv.second);
  }

  return roles;
}

void    
MotionCondition::
ReCenter(const std::vector<double>& _t) {

  if(m_explicit)
    return;

  m_translatedConstraints.clear();

  for(const auto& constraint : m_constraints) {
    auto boundary = constraint.second->GetBoundary()->Clone();
    boundary->Translate(_t);

    auto c = std::unique_ptr<Constraint>(new BoundaryConstraint(
             constraint.second->GetRobot(),std::move(boundary)));
    m_translatedConstraints.push_back(std::make_pair(constraint.first,std::move(c)));
    m_roles[m_translatedConstraints.back().second.get()] = m_roles[constraint.second.get()];
  }
}

Constraint*
MotionCondition::
GetRoleConstraint(const std::string& _role) const {

  auto iter = m_roleConstraints.find(_role);
  if(iter == m_roleConstraints.end())
    return nullptr;

  return iter->second;
}

/*--------------------- Helper Functions ---------------------*/

/*------------------------------------------------------------*/
