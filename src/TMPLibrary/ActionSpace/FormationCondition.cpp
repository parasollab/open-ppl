#include "FormationCondition.h"

#include "ConfigurationSpace/Formation.h"

#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include "TMPLibrary/TMPLibrary.h"

#include "EulerAngle.h"
#include "Orientation.h"


/*----------------------- Construction -----------------------*/

FormationCondition::
FormationCondition() {
  this->SetName("FormationCondition");
}

FormationCondition::
FormationCondition(XMLNode& _node, TMPLibrary* _tmpLibrary) : Condition(_node,_tmpLibrary) {
  this->SetName("FormationCondition");

  for(auto& child : _node) {
    if(child.Name() == "Role") {
    
      Role role;

      role.name = child.Read("name", true, "", 
                        "Type of robot required in formation.");
      role.type = child.Read("type", true, "", 
                        "Type of robot required in formation.");

      if(child.begin() != child.end()) {
        role.leader = false;
      }

      for(auto& grandchild : child) {
        if(grandchild.Name() == "FormationConstraint") {
          role.referenceRole = grandchild.Read("refRole", true, "", 
                    "The role in the formation that this constraint is dependent on.");

          role.referenceBody = grandchild.Read("refBody", true, 0,0,100,
                    "The body index in the reference role robot that this constraint is "
                    "dependent on.");

          role.dependentBody = grandchild.Read("depBody", true, 0,0,100,
                    "The body index in this role robot that this is subject to this "
                    "constraint.");

          std::string translationString = grandchild.Read("translation", true, "",
                    "The translation portion of the formation constraint.");

          auto translation = ParseVectorString(translationString);

          std::string orientationString = grandchild.Read("translation", true, "",
                    "The translation portion of the formation constraint.");

          auto eulerVec = ParseVectorString(orientationString);
          
          role.transformation = Transformation(Vector3d(translation[0],
                                                        translation[1],
                                                        translation[2]),
                                               EulerAngle(eulerVec[0],
                                                          eulerVec[1],
                                                          eulerVec[2]));

        }
        else {
          throw ParseException(WHERE) << "Unsupported XML Name.";
        }
      }

      m_roles[role.name] = role;
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

Formation*
FormationCondition::
GenerateFormation(std::unordered_map<std::string,Robot*>& _roleMap) {

  Robot* leader = nullptr;
  std::vector<Robot*> robots;
  std::unordered_map<MultiBody*,Formation::FormationConstraint> constraintMap;

  for(auto kv : m_roles) {
    auto role = kv.second;

    // Make sure the role is satisfied.
    auto robot = _roleMap.at(role.name);
    if(!robot) {
      throw RunTimeException(WHERE) << "Cannot create formation because input "
                                       "role map is missing members.";
    }

    robots.push_back(robot);

    // Check if this robot serves as the leader.
    if(role.leader) {
      leader = robot;
      continue;
    }
   
    // Make sure the reference robot for the formation constraint is assigned. 
    auto referenceRobot = _roleMap.at(role.referenceRole);
    if(!referenceRobot) {
      throw RunTimeException(WHERE) << "Cannot create formation because input "
                                       "role map is missing members.";
    }

    // Convert role info into formation constraint.
    Formation::FormationConstraint constraint;
    constraint.dependentRobot = robot;
    constraint.dependentBody = robot->GetMultiBody()->GetBody(role.dependentBody);
    constraint.referenceRobot = referenceRobot;
    constraint.referenceBody = referenceRobot->GetMultiBody()->GetBody(role.referenceBody);
    constraint.transformation = role.transformation;

    constraintMap[robot->GetMultiBody()] = constraint;
  }

  auto formation = new Formation(robots,leader,constraintMap);
  return formation;
}

/*------------------------ Accessors -------------------------*/
    
const std::vector<std::string> 
FormationCondition::
GetRoles() const {
  std::vector<std::string> roles;
  for(auto role : m_roles) {
    roles.push_back(role.first);
  }

  return roles;
}

/*--------------------- Helper Functions ---------------------*/

bool
FormationCondition::
CheckRequirements(RobotGroup* _group) const {
  std::set<Robot*> used;

  // Check if the robot group meets all of the constraints
  for(const auto& role : m_roles) {
    auto type = role.second.type;

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

std::vector<double>
FormationCondition::
ParseVectorString(std::string _s) {

  std::vector<double> vec;
  std::istringstream stream(_s);

  for(size_t i = 0; i < 3; i++) {
    double d;
    stream >> d;
    vec.push_back(d);
  }

  return vec;
}

/*------------------------------------------------------------*/
