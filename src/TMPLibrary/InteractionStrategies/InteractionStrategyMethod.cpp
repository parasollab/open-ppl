#include "InteractionStrategyMethod.h"

#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/MPSolution.h"
#include "MPLibrary/Samplers/SamplerMethod.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/GroupTask.h"
#include "MPProblem/MPProblem.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"
#include "TMPLibrary/ActionSpace/ProximityCondition.h"
#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"

/*--------------------------------------- Construction ---------------------------------*/

InteractionStrategyMethod::
InteractionStrategyMethod(XMLNode& _node) : TMPBaseObject(_node) {
  m_sgLabel     = _node.Read("sgLabel", false, "", 
                             "StateGraph to use within strategy.");
  m_smLabel     = _node.Read("sgLabel", true, "", 
                             "Sampler method to use within strategy.");
  m_numNodes    = _node.Read("numNodes", false, 1, 0, 100, 
                             "Numbner of samples for finding constraints.");
  m_maxAttempts = _node.Read("maxAttempts", false, 10, 0, 100, 
                             "Numbner of sample attempts for finding constraints.");
}

InteractionStrategyMethod::
~InteractionStrategyMethod() { }

/*--------------------------------------- Interface ---------------------------------*/

bool 
InteractionStrategyMethod::
operator()(Interaction* _interaction, State& _start) {
  return false;
}

/*------------------------------------ Helper Functions -----------------------------*/

void
InteractionStrategyMethod::
AssignRoles(const State& _state, const std::vector<std::string>& _conditions) {

  auto as = this->GetTMPLibrary()->GetActionSpace();

  // Find relevant conditions
  std::unordered_map<RobotGroup*,std::vector<Condition*>> conditionMap;
 
  for(auto label : _conditions) {
    auto condition = as->GetCondition(label);

    auto m = dynamic_cast<MotionCondition*>(condition);
    if(!m)
      continue;

    auto group = condition->Satisfied(_state);
    if(!group)
      throw RunTimeException(WHERE) << "Interaction conditions "
                                    << "not satisfied by input state.";

    conditionMap[group].push_back(condition);
  }

  // Identify roles
  std::set<Robot*> usedRobots;

  for(auto kv : conditionMap) {
    auto group = kv.first;
    auto robots = group->GetRobots();
    auto conditions = kv.second;

    auto groupRoadmap = _state.at(group).first;
    auto groupVID = _state.at(group).second;

    auto groupCfg = groupRoadmap->GetVertex(groupVID);

    for(auto condition : conditions) {
      // Check if condition is a motion condition
      auto m = dynamic_cast<MotionCondition*>(condition);
      if(!m) 
        continue;

      // Match constraints to robots
      const auto& constraints = m->GetConstraints();
      for(const auto& c : constraints) {

        auto role = m->GetRole(c.second);
        // Find the right robot for the role
        auto type = c.first;
        for(auto robot : robots) {
          // Check that robot is of the right type
          if(robot->GetCapability() != type) 
            continue;

          // Check if robot has been used already
          if(usedRobots.count(robot))
            continue;

          // Check that robot cfg matches the constraint
          auto cfg = groupCfg.GetRobotCfg(robot);
          if(!c.second->Satisfied(cfg)) 
            continue;

          m_roleMap[role] = robot;
          usedRobots.insert(robot);
        }
      }
    }
  } 
}

void
InteractionStrategyMethod::
SetInteractionBoundary(Interaction* _interaction, const State& _state) {

  auto as = this->GetTMPLibrary()->GetActionSpace();

  // Look for the proximity condition and set the interaction boundary.
  for(const auto label : _interaction->GetPreConditions()) {
    auto condition = as->GetCondition(label);
    auto p = dynamic_cast<ProximityCondition*>(condition);
    if(!p)
      continue;

    m_boundary = p->GetBoundary(_state)->Clone();
    return;
  }

  // If no proximity condition is found, use the environment boundary.
  m_boundary = this->GetMPProblem()->GetEnvironment()->GetBoundary()->Clone();
}
    
std::unordered_map<Robot*,Constraint*>
InteractionStrategyMethod::
GenerateConstraints(const State& _state) {
  std::unordered_map<Robot*,Constraint*> constraintMap;

  for(auto kv : _state) {
    auto group = kv.first;
    auto grm = kv.second.first;
    auto vid = kv.second.second;
    auto gcfg = grm->GetVertex(vid);

    for(auto robot : group->GetRobots()) {
      auto cfg = gcfg.GetRobotCfg(robot);
      auto constraint = new CSpaceConstraint(robot,cfg);
      constraintMap[robot] = constraint;
    }
  }

  return constraintMap;
}

std::unordered_map<Robot*,Constraint*> 
InteractionStrategyMethod::
GenerateConstraints(const std::vector<std::string>& _conditions, 
                    const std::vector<RobotGroup*>& _groups) {

  std::unordered_map<Robot*,Constraint*> constraintMap;
  auto as = this->GetTMPLibrary()->GetActionSpace();
  
  for(const auto label : _conditions) {
    auto constraint = as->GetCondition(label);

    // Check if the condition is a motion condition.
    auto m = dynamic_cast<MotionCondition*>(constraint);
    if(!m)
      continue;

    // Center constraint on boundary.
    auto center = m_boundary->GetCenter();
    m->ReCenter(center);

    // Match constraints to robots.
    const auto& constraints = m->GetConstraints();
    for(const auto c : constraints) {

      // Check if the costraint role has been assigned.
      auto role = m->GetRole(c.second);
      auto iter = m_roleMap.find(role);
      if(iter == m_roleMap.end())
        throw RunTimeException(WHERE) << "Role "
                                      << role
                                      << " is unassigned.";
      
      auto robot = m_roleMap[role];
      constraintMap[robot] = c.second;
    }
  }

  // Ensure that each group is either entirely defined or undefined.
  std::vector<RobotGroup*> unconstrainedGroups;
  for(const auto group : _groups) {
    bool constrained = false;
    for(auto robot : group->GetRobots()) {
      auto iter = constraintMap.find(robot);
      auto exists = iter == constraintMap.end();
      if(exists)
        constrained = true;
      if(exists != constrained)
        throw RunTimeException(WHERE) << "Group "
                                      << group->GetLabel()
                                      << " is only partially constrained.";
    }
    if(!constrained)
      unconstrainedGroups.push_back(group);
  }

  // If all groups are constrained, return the constraint map.
  if(unconstrainedGroups.empty())
    return constraintMap;

  // Sample Motion Constraints for unconstrained groups.
  auto constraints = SampleMotionConstraints(_conditions,unconstrainedGroups);
  
  // Make sure that valid constraints were found.
  if(constraints.empty())
    return {};

  for(auto constraint : constraints) {
    constraintMap[constraint.first] = constraint.second;
  }

  return constraintMap;
}

std::unordered_map<Robot*,Constraint*> 
InteractionStrategyMethod::
SampleMotionConstraints(const std::vector<std::string>& _conditions,
                        const std::vector<RobotGroup*> _groups) {

  std::unordered_map<Robot*,Constraint*> constraintMap;
  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto hcr = dynamic_cast<CombinedRoadmap*>(this->GetStateGraph(m_sgLabel).get());

  for(const auto& label : _conditions) {
    auto condition = as->GetCondition(label);
    // Check if the condition is a formation condition.
    auto f = dynamic_cast<FormationCondition*>(condition);
    if(!f)
      continue;

    auto roles = f->GetRoles();
    std::vector<Robot*> robots;
    std::string groupLabel = "";
    std::unordered_map<std::string,Robot*> roleMap;
    for(auto role : roles) {
      auto robot = m_roleMap[role];
      robots.push_back(robot);
      groupLabel += (role + ":" + robot->GetLabel() + "--");
      roleMap[role] = robot;
    }
    groupLabel += std::to_string(LRand());

    RobotGroup* group = this->GetMPProblem()->AddRobotGroup(robots,groupLabel);
    
    // Check if this is one of the input groups.
    bool match = false;
    for(const auto& g : _groups) {
      if(group == g) {
        match = true;
        break;
      }
    }
    if(!match)
      continue;
    
    // Check if group already exists and update MPSolution.
    if(group->GetLabel() == groupLabel) { 
      hcr->AddRobotGroup(group);
    }

    // Generate a dummy solution and task to utilize the MPLibrary.
    auto formation = f->GenerateFormation(roleMap);
    MPSolution sol(group);
    sol.GetGroupRoadmap(group)->AddFormation(formation);
    sol.GetGroupRoadmap(group)->SetFormationActive(formation);
    GroupTask task(group);

    auto lib = this->GetTMPLibrary()->GetMPLibrary();
    lib->SetMPSolution(&sol);
    lib->SetGroupTask(&task);

    // Sample configurations for the group in the interaction boundary.
    auto sampler = lib->GetSampler(m_smLabel);
    std::vector<GroupCfg> samples;
    sampler->Sample(m_numNodes,m_maxAttempts,m_boundary.get(),
                    std::back_inserter(samples));
    
    if(samples.empty())
      return {};

    for(auto robot : group->GetRobots()) {
      auto cfg = samples[0].GetRobotCfg(robot);
      auto constraint = new CSpaceConstraint(robot,cfg);
      constraintMap[robot] = constraint;
    }
  }

  return constraintMap;
}

void
InteractionStrategyMethod::
SetActiveFormations(std::vector<std::string> _conditions, MPSolution* _solution) {
  
  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto prob = this->GetMPProblem();
 
  // Map of group roadmaps to active formations
  std::unordered_map<GroupRoadmap<GroupCfg,GroupLocalPlan<Cfg>>*,
                     std::vector<Formation*>> roadmapFormations;

  for(auto label : _conditions) {
    // Make sure it's a formation condition.
    auto condition = as->GetCondition(label);
    auto f = dynamic_cast<FormationCondition*>(condition);
    if(!f)
      continue;

    // Make sure there are robots to actually make a formation with.
    if(f->GetRoles().size() < 2)
      continue;

    auto formation = f->GenerateFormation(m_roleMap);

    // Collect robots from assigned roles
    std::vector<Robot*> robots = formation->GetRobots();
    std::string groupLabel = "";

    for(auto role : f->GetRoles()) {
      auto robot = m_roleMap[role];
      //robots.push_back(robot);
      groupLabel += (role + ":" + robot->GetLabel() + "--");
    }

    // Intialize group
    RobotGroup* group = prob->AddRobotGroup(robots,groupLabel);

    for(auto robot : group->GetRobots()) {
      _solution->AddRobot(robot);
    }

    _solution->AddRobotGroup(group);
    auto grm = _solution->GetGroupRoadmap(group);
    grm->SetAllFormationsInactive();

    grm->AddFormation(formation);
    roadmapFormations[grm].push_back(formation);
  }
  
  for(auto& kv : roadmapFormations) {
    auto grm = kv.first;
    auto formations = kv.second;
    for(auto& formation : formations) {
      grm->SetFormationActive(formation);
    }
  }
}
std::vector<Path*> 
InteractionStrategyMethod::
DecouplePath(MPSolution* _solution, GroupPathType* _groupPath) {
  return {};
}

