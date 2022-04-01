#include "InteractionStrategyMethod.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/MPSolution.h"
#include "MPLibrary/Samplers/SamplerMethod.h"
#include "MPLibrary/ValidityCheckers/ValidityCheckerMethod.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/GroupTask.h"
#include "MPProblem/MPProblem.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"
#include "TMPLibrary/ActionSpace/ProximityCondition.h"
#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"

/*--------------------------------------- Construction ---------------------------------*/

InteractionStrategyMethod::
InteractionStrategyMethod(XMLNode& _node) : TMPBaseObject(_node) {
  m_sgLabel     = _node.Read("sgLabel", true, "", 
                             "StateGraph to use within strategy.");
  m_smLabel     = _node.Read("smLabel", true, "", 
                             "Sampler method to use within strategy.");
  m_vcLabel     = _node.Read("vcLabel", true, "", 
                             "Validity checker to use within strategy.");                           
  m_numNodes    = _node.Read("numNodes", false, 1, 1, MAX_INT, 
                             "Numbner of samples for finding constraints.");
  m_maxAttempts = _node.Read("maxAttempts", false, 100, 1, MAX_INT, 
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

  std::vector<Condition*> formationConditions;
  std::vector<Condition*> motionConditions;

  for(auto label1 : _conditions) {

    // Check that condition is a formation condition.
    auto c1 = as->GetCondition(label1);
    auto f = dynamic_cast<FormationCondition*>(c1);
    if(!f)
      continue;

    bool match = false;

    for(auto label2 : _conditions) {

      // Check if condition is a motion condition.
      auto c2 = as->GetCondition(label2);
      auto m = dynamic_cast<MotionCondition*>(c2);
      if(!m)
        continue;

      // Make sure group is not already assigned to a formation.
      if(std::count(motionConditions.begin(),motionConditions.end(),m))
        continue;

      // Check if the motion condition matches the formation condition.
      if(CompareConditionRoleSets(f,m)) { 
        // Add it to the motion conditions if it's a match
        motionConditions.push_back(c2); 
        match = true;
      }
    }
    
    // If no motion condition was found, add the formation condition.
    if(!match)
      formationConditions.push_back(c1); 
  }

  std::unordered_set<RobotGroup*> usedGroups;
  AssignRolesFromConditions(_state,motionConditions,usedGroups);
  AssignRolesFromConditions(_state,formationConditions,usedGroups);
}

void
InteractionStrategyMethod::
AssignRolesFromConditions(const State& _state,
                          const std::vector<Condition*>& _conditions,
                          std::unordered_set<RobotGroup*>& _usedGroups) {

  for(auto condition : _conditions) {
    bool satisfied = false;
    for(auto kv : _state) {

      // Check if the group has been used already.
      auto group = kv.first;
      if(_usedGroups.count(group))
        continue;
      
      // Create an isolated state for the group.
      State state;
      state[kv.first] = kv.second;

      // Check if the group satisfies the condition.
      if(!condition->Satisfied(state))
        continue;

      satisfied = true;

      // Claim the group for this condition.
      _usedGroups.insert(group);
      condition->AssignRoles(m_roleMap,state);
    }

    if(!satisfied)
      throw RunTimeException(WHERE) << "No satisfying group found to assign roles for "
                                    << condition->GetLabel() << ".";
  }
}

void
InteractionStrategyMethod::
SetInteractionBoundary(Interaction* _interaction, const State& _state) {

  auto as = this->GetTMPLibrary()->GetActionSpace();

  // Look for the proximity condition and set the interaction boundary.
  const auto& initialStage = _interaction->GetStages()[0];
  for(const auto label : _interaction->GetStageConditions(initialStage)) {
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

      if(m_debug) {
        std::cout << "Generating constraint for " << robot->GetLabel() 
                  << " at " << cfg.PrettyPrint() << std::endl;
      }

      auto constraint = new CSpaceConstraint(robot,cfg);
      constraintMap[robot] = constraint;
    }
  }

  return constraintMap;
}

std::unordered_map<Robot*,Constraint*> 
InteractionStrategyMethod::
GenerateConstraints(const std::vector<std::string>& _conditions, 
                    const std::vector<RobotGroup*>& _groups,
                    const State& _state, const std::set<Robot*>& _staticRobots) {

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

      // Check if the constraint role has been assigned.
      auto role = m->GetRole(c.second);
      auto iter = m_roleMap.find(role);
      if(iter == m_roleMap.end())
        throw RunTimeException(WHERE) << "Role "
                                      << role
                                      << " is unassigned.";
      
      auto robot = m_roleMap[role];

      if(_staticRobots.count(robot))
        throw RunTimeException(WHERE) << "Attempting to give static robot a different constraint.";

      constraintMap[robot] = c.second;
    }
  }

  // Generate constraints for static robots at their current state
  /*State staticState;
  for(const auto& group : _groups) {
    // Check that whole group is static or not.
    const auto& robots = group->GetRobots();
    bool s = _staticRobots.count(robots[0]);
    for(size_t i = 1; i < robots.size(); i++) {
      if(s != _staticRobots.count(robots[i]))
        throw RunTimeException(WHERE) << "Group has inconsistent static status.";
    }

    if(s)
      staticState[group] = _state.at(group);
  }

  auto staticConstraints = GenerateConstraints(staticState);

  // Copy constraints to main constraint map.
  for(auto kv : staticConstraints) {
    constraintMap[kv.first] = kv.second;
  }*/

  // Ensure that each group is either entirely defined or undefined.
  std::vector<Robot*> unconstrainedRobots;
  for(const auto group : _groups) {
    bool constrained = false;
    for(auto robot : group->GetRobots()) {
      auto iter = constraintMap.find(robot);
      auto exists = iter != constraintMap.end();
      if(exists)
        constrained = true;
      if(exists != constrained)
        throw RunTimeException(WHERE) << "Group "
                                      << group->GetLabel()
                                      << " is only partially constrained.";
    }
    if(!constrained) {
      for(auto robot : group->GetRobots()) {
        unconstrainedRobots.push_back(robot);
      }
    }
  }

  // If all groups are constrained, return the constraint map.
  if(unconstrainedRobots.empty())
    return constraintMap;

  // Sample Motion Constraints for unconstrained groups.
  auto constraints = SampleMotionConstraints(_conditions,unconstrainedRobots,
                                             _state, _staticRobots);
  
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
GenerateConstraintsDependent(const std::vector<std::string>& _conditions, 
                    const std::vector<RobotGroup*>& _groups,
                    const State& _state, const std::set<Robot*>& _staticRobots) {

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

      // Check if the constraint role has been assigned.
      auto role = m->GetRole(c.second);
      auto iter = m_roleMap.find(role);
      if(iter == m_roleMap.end())
        throw RunTimeException(WHERE) << "Role "
                                      << role
                                      << " is unassigned.";
      
      auto robot = m_roleMap[role];

      if(_staticRobots.count(robot))
        throw RunTimeException(WHERE) << "Attempting to give static robot a different constraint.";

      constraintMap[robot] = c.second;
    }
  }

  // Generate constraints for static robots at their current state
  /*State staticState;
  for(const auto& group : _groups) {
    // Check that whole group is static or not.
    const auto& robots = group->GetRobots();
    bool s = _staticRobots.count(robots[0]);
    for(size_t i = 1; i < robots.size(); i++) {
      if(s != _staticRobots.count(robots[i]))
        throw RunTimeException(WHERE) << "Group has inconsistent static status.";
    }

    if(s)
      staticState[group] = _state.at(group);
  }

  auto staticConstraints = GenerateConstraints(staticState);

  // Copy constraints to main constraint map.
  for(auto kv : staticConstraints) {
    constraintMap[kv.first] = kv.second;
  }*/

  // Ensure that each group is either entirely defined or undefined.
  std::vector<Robot*> unconstrainedRobots;
  for(const auto group : _groups) {
    bool constrained = false;
    for(auto robot : group->GetRobots()) {
      auto iter = constraintMap.find(robot);
      auto exists = iter != constraintMap.end();
      if(exists)
        constrained = true;
      if(exists != constrained)
        throw RunTimeException(WHERE) << "Group "
                                      << group->GetLabel()
                                      << " is only partially constrained.";
    }
    if(!constrained) {
      for(auto robot : group->GetRobots()) {
        unconstrainedRobots.push_back(robot);
      }
    }
  }

  // If all groups are constrained, return the constraint map.
  if(unconstrainedRobots.empty())
    return constraintMap;

  // Sample Motion Constraints for unconstrained groups.
  auto constraints = SampleMotionConstraintsDependent(_conditions,unconstrainedRobots,
                                             _state, _staticRobots);
  
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
SampleMotionConstraintsDependent(const std::vector<std::string>& _conditions,
                        const std::vector<Robot*> _robots,
                        const State& _state, 
                        const std::set<Robot*>& _staticRobots) {

  std::unordered_map<Robot*,Constraint*> constraintMap;
  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto hcr = dynamic_cast<CombinedRoadmap*>(this->GetStateGraph(m_sgLabel).get());
  CSpaceBoundingSphere* handoffBounds;
  double HandoffDistance = 5;
  bool FirstRun = true;
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
    
    // Check if this is consists of input robots
    bool match = true;
    for(auto robot : group->GetRobots()) {
      if(std::count(_robots.begin(),_robots.end(),robot)) {
        continue;
      }
      match = false;
      break;
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

    if(formation) {
      sol.GetGroupRoadmap(group)->AddFormation(formation);
      sol.GetGroupRoadmap(group)->SetFormationActive(formation);
    }

    GroupTask task(group);

    auto lib = this->GetTMPLibrary()->GetMPLibrary();
    lib->SetMPSolution(&sol);
    lib->SetGroupTask(&task);

    // TODO::Currently only helpful if the leader is static. Any
    // child that is static in a formation with an active leader
    // will not be accounted for. This can be worked around by
    // creating stages with different formations and thus different
    // leaders, but this is an inelegant solution and should be fixed.

    // Check if the formation is defined by a static leader.
    Boundary* boundary;
    auto leader = formation ? formation->GetLeader() : group->GetRobots()[0];

    // Find group in input state with leader, because leader is necessary for dependent
    //This block is no longer in an if block
    if(_staticRobots.count(leader)) {
      // Find group in input state with leader
      bool contains = false;
      for(auto kv : _state) {
        auto g = kv.first;
        for(auto r : g->GetRobots()) {
          if(r == leader) {
            contains = true;
            break;
          }
        }

        // These are not the droids you're looking for.
        if(!contains)
          continue;

        auto grm = kv.second.first;
        auto vid = kv.second.second;
        auto gcfg = grm->GetVertex(vid);
        auto cfg = gcfg.GetRobotCfg(leader);
        auto c = new CSpaceBoundingBox(cfg.DOF());
        c->ShrinkToPoint(cfg);
        boundary = c;
        break;
      }

      if(!contains)
        throw RunTimeException(WHERE) << "Failed to find current state of leader.";
    }
    else {
      boundary = m_boundary.get();
    }
    // else {
    //   boundary = m_boundary.get();
    // }
    
    // BoundaryMap* BoundMap;
    // for (auto robot : group->GetRobots()){
    //   auto cfg(this->GetT)
    //     if (robot==leader) BoundMap[robot]=boundary;
    //     else BoundMap[robot] = handoffBounds;
    // }
    Boundary* Bounds = FirstRun ? boundary : handoffBounds;
    auto sampler = lib->GetSampler(m_smLabel);
    std::vector<GroupCfg> samples;
    // Sample configurations for the group in the boundary.
    sampler->Sample(m_numNodes,m_maxAttempts,Bounds,
                    std::back_inserter(samples));
    
    // Delete CSpace boundary if necessary
    if(_staticRobots.count(leader)) {
      delete boundary;
    }

    if(samples.empty())
      return {};

    auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

    bool foundGoal = false;

    for(auto gcfg : samples) {
      // Check if group cfg is valid, otherwise look at the next one
      if(!vc->IsValid(gcfg,this->GetNameAndLabel()))
        continue;
      foundGoal = true;
      //Because these are not manipulators it is easier to find central point to group
      size_t init = 0;
      Cfg Center = gcfg.GetRobotCfg(init);
      bool firstRobotinGroup = true;
      for(auto robot : group->GetRobots()) {

        auto cfg = gcfg.GetRobotCfg(robot);
        if (firstRobotinGroup) firstRobotinGroup=false;
        else Center+=cfg;
        if(m_debug) {
          std::cout << "Generating constraint for " << robot->GetLabel() 
                    << " at " << cfg.PrettyPrint() << std::endl;
        }

        auto constraint = new CSpaceConstraint(robot,cfg);
        constraintMap[robot] = constraint;
      }
      Center/=gcfg.GetNumRobots();
      //Make a bounding sphere around this groups center?
      if (FirstRun){
        
        handoffBounds = new CSpaceBoundingSphere(Center.GetData(), HandoffDistance);
      }
      // break because we've already found a valid goal
      break;
    }
    // If not goal was found, return an empty map
    if(!foundGoal)
      return {};
  }

  return constraintMap;
}

std::unordered_map<Robot*,Constraint*> 
InteractionStrategyMethod::
SampleMotionConstraints(const std::vector<std::string>& _conditions,
                        const std::vector<Robot*> _robots,
                        const State& _state, 
                        const std::set<Robot*>& _staticRobots) {

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
    
    // Check if this is consists of input robots
    bool match = true;
    for(auto robot : group->GetRobots()) {
      if(std::count(_robots.begin(),_robots.end(),robot)) {
        continue;
      }
      match = false;
      break;
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

    if(formation) {
      sol.GetGroupRoadmap(group)->AddFormation(formation);
      sol.GetGroupRoadmap(group)->SetFormationActive(formation);
    }

    GroupTask task(group);

    auto lib = this->GetTMPLibrary()->GetMPLibrary();
    lib->SetMPSolution(&sol);
    lib->SetGroupTask(&task);

    // TODO::Currently only helpful if the leader is static. Any
    // child that is static in a formation with an active leader
    // will not be accounted for. This can be worked around by
    // creating stages with different formations and thus different
    // leaders, but this is an inelegant solution and should be fixed.

    // Check if the formation is defined by a static leader.
    Boundary* boundary;
    auto leader = formation ? formation->GetLeader() : group->GetRobots()[0];

    if(_staticRobots.count(leader)) {
      // Find group in input state with leader
      bool contains = false;
      for(auto kv : _state) {
        auto g = kv.first;
        for(auto r : g->GetRobots()) {
          if(r == leader) {
            contains = true;
            break;
          }
        }

        // These are not the droids you're looking for.
        if(!contains)
          continue;

        auto grm = kv.second.first;
        auto vid = kv.second.second;
        auto gcfg = grm->GetVertex(vid);
        auto cfg = gcfg.GetRobotCfg(leader);
        auto c = new CSpaceBoundingBox(cfg.DOF());
        c->ShrinkToPoint(cfg);
        boundary = c;
        break;
      }

      if(!contains)
        throw RunTimeException(WHERE) << "Failed to find current state of leader.";
    }
    else {
      boundary = m_boundary.get();
    }

    // Sample configurations for the group in the boundary.
    auto sampler = lib->GetSampler(m_smLabel);
    std::vector<GroupCfg> samples;
    sampler->Sample(m_numNodes,m_maxAttempts,boundary,
                    std::back_inserter(samples));
    
    // Delete CSpace boundary if necessary
    if(_staticRobots.count(leader)) {
      delete boundary;
    }

    if(samples.empty())
      return {};

    auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

    bool foundGoal = false;

    for(auto gcfg : samples) {
      // Check if group cfg is valid, otherwise look at the next one
      if(!vc->IsValid(gcfg,this->GetNameAndLabel()))
        continue;
      foundGoal = true;
      for(auto robot : group->GetRobots()) {

        auto cfg = gcfg.GetRobotCfg(robot);

        if(m_debug) {
          std::cout << "Generating constraint for " << robot->GetLabel() 
                    << " at " << cfg.PrettyPrint() << std::endl;
        }

        auto constraint = new CSpaceConstraint(robot,cfg);
        constraintMap[robot] = constraint;
      }
      // break because we've already found a valid goal
      break;
    }
    // If not goal was found, return an empty map
    if(!foundGoal)
      return {};
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
  auto grm = _groupPath->GetRoadmap();
  auto group = grm->GetGroup();
  auto robots = group->GetRobots();
 
  std::unordered_map<Robot*,std::vector<size_t>> individualVIDs;
 
  auto& gcfgs = _groupPath->Cfgs();
  for(auto& gcfg : gcfgs) {
    for(auto robot : robots) {
      auto vid = gcfg.GetVID(robot);
      individualVIDs[robot].push_back(vid);
    }  
  }

  std::vector<Path*> paths;

  for(auto robot : robots) {
    auto path = _solution->GetPath(robot);
    path->Clear();
    *path += individualVIDs[robot];
    path->SetTimeSteps(_groupPath->TimeSteps());
    paths.push_back(path);
  }

  return paths;
}

bool
InteractionStrategyMethod::
CompareConditionRoleSets(FormationCondition* _f,
                         MotionCondition* _m) {
  auto r1 = _f->GetRoles();
  auto r2 = _m->GetRoles();
  if(r1.size() != r2.size())
    return false;

  for(auto role : r1) {
    if(!r2.count(role))
      return false;
  }

  return true;
}

std::set<Robot*>
InteractionStrategyMethod::
GetStaticRobots(const std::vector<std::string>& _conditions) {

  auto as = this->GetTMPLibrary()->GetActionSpace();

  std::set<Robot*> staticRobots;

  for(const auto& label : _conditions) {
    auto condition = as->GetCondition(label);
    auto f = dynamic_cast<FormationCondition*>(condition);

    if(!f or !f->IsStatic())
      continue;

    const auto& roles = f->GetRoles();
    for(const auto& role : roles) {
      auto robot = m_roleMap[role];
      staticRobots.insert(robot);
    }        
  }

  return staticRobots;
}

//function added so that I can output the maps for every robot
std::set<Robot*>
InteractionStrategyMethod::
GetRobots(const std::vector<std::string>& _conditions) {

  auto as = this->GetTMPLibrary()->GetActionSpace();

  std::set<Robot*> Robots;

  for(const auto& label : _conditions) {
    auto condition = as->GetCondition(label);
    auto f = dynamic_cast<FormationCondition*>(condition);

    const auto& roles = f->GetRoles();
    for(const auto& role : roles) {
      auto robot = m_roleMap[role];
      Robots.insert(robot);
    }        
  }
  return Robots;
}

    
void
InteractionStrategyMethod::
ConfigureStaticRobots(const std::set<Robot*>& _staticRobots, const State& _state) {
  auto prob = this->GetTMPLibrary()->GetMPProblem();

  for(const auto& robot : prob->GetRobots()) {
    robot->SetVirtual(true);
  }

  for(const auto& kv : _state) {
    auto group = kv.first;
    auto grm = kv.second.first;
    auto vid = kv.second.second;
    auto gcfg = grm->GetVertex(vid);

    for(auto robot : group->GetRobots()) {
      if(!_staticRobots.count(robot))
        continue;

      robot->SetVirtual(false);
      auto cfg = gcfg.GetRobotCfg(robot);
      robot->GetMultiBody()->Configure(cfg);
    }
  }
}

void
InteractionStrategyMethod::
ResetStaticRobots() {
  auto prob = this->GetTMPLibrary()->GetMPProblem();

  for(const auto& robot : prob->GetRobots()) {
    if(this->GetPlan()->GetCoordinator()->GetRobot() == robot.get())
      continue;
    robot->SetVirtual(false);
  }
}

void
InteractionStrategyMethod::
MoveStateToLocalSolution(Interaction* _interaction, State& _state) {
  auto stage = _interaction->GetStages()[1];
  auto solution = _interaction->GetToStageSolution(stage);
  
  for(auto& kv : _state) {
    // Grab group cfg from original roadmap
    auto group = kv.first;
    auto grm = kv.second.first;
    auto vid = kv.second.second;
    auto gcfg = grm->GetVertex(vid);

    // Add group cfg to new roadmap
    solution->AddRobotGroup(group);
    grm = solution->GetGroupRoadmap(group);

    for(auto form : gcfg.GetFormations()) {
      grm->AddFormation(form,true);
    }

    auto newGcfg = gcfg.SetGroupRoadmap(grm);
    vid = grm->AddVertex(newGcfg);

    // Update values in the state
    kv.second = std::make_pair(grm,vid);
  }
}
    
void
InteractionStrategyMethod::
SampleStartState(Interaction* _interaction, State& _state) {
  throw RunTimeException(WHERE) << "Not yet implemented" << std::endl;
  //TODO::Sample a start state
}
