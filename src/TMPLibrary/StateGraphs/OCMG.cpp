#include "OCMG.h"

#include "Behaviors/Agents/Coordinator.h"

#include "ConfigurationSpace/Cfg.h"

#include "Geometry/Boundaries/CSpaceBoundingBox.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"
#include "MPProblem/MPTask.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"
#include "TMPLibrary/InteractionStrategies/InteractionStrategyMethod.h"
#include "TMPLibrary/Solution/Plan.h"

/*------------------------------- Construction -------------------------------*/
OCMG::
OCMG() {
  this->SetName("OCMG");
}

OCMG::
OCMG(XMLNode& _node) : StateGraph(_node) { 
  this->SetName("OCMG");
  m_roadmapStrategy = _node.Read("roadmapStrategy",true,"",
          "Strategy to use to build individual robot roadmaps.");

  m_querySampler = _node.Read("querySampler",true,"",
          "Sampler method to query goal configuration.");

  m_maxQueryAttempts = _node.Read("maxQueryAttempts",false,
          10,1,MAX_INT,"Max number of attempts to query goal configuration.");
    
  m_interactionAttempts = _node.Read("interactionAttempts",false,
          10,1,MAX_INT,"Max number of attempts to sample an interaction.");

  m_interactionSamples = _node.Read("interactionSamples",false,
          1,1,MAX_INT,"Number of samples to take for each interaction.");

  m_connector = _node.Read("connector",true,"",
          "Connector to use to connect interactions to roadmaps.");
}

/*---------------------------- Initialization --------------------------------*/

void
OCMG::
Initialize() {

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Initialize");

  auto coordinator = plan->GetCoordinator();

  // Initialize solution object.
  m_solution = std::unique_ptr<MPSolution>(
        new MPSolution(coordinator->GetRobot()));


  // Build roadmaps for each robot
  for(auto& kv : coordinator->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive()) {
        ConstructObjectRoadmap(robot);
        continue;
      }

      ConstructRobotRoadmap(robot);
    }
  }

  SampleInteractions();

  BuildIndividualObjectModeGraph();
}

/*------------------------------- Accessors ----------------------------------*/

OCMG::MPSolution*
OCMG::
GetMPSolution() {
  return m_solution.get();
}

OCMG::GroupRoadmapType*
OCMG::
GetGroupRoadmap(RobotGroup* _group) {
  return m_solution->GetGroupRoadmap(_group);
}

OCMG::SingleObjectModeGraph*
OCMG::
GetSingleObjectModeGraph() {
  return m_omg.get();
}

const OCMG::SavedInteractions&
OCMG::
GetSavedInteractions() {
  return m_savedInteractions;
}

std::vector<RobotGroup*>
OCMG::
GetRobotGroups() {
  std::vector<RobotGroup*> groups;
  for(auto group : m_groups)
    groups.push_back(group);

  return groups;
}

std::vector<Robot*>
OCMG::
GetRobots() {
  std::vector<Robot*> robots;
  for(auto robot : m_robots)
    robots.push_back(robot);

  return robots;
}

std::vector<Robot*>
OCMG::
GetObjects() {
  std::vector<Robot*> objects;
  for(auto object : m_objects)
    objects.push_back(object);

  return objects;
}

OCMG::TerrainVIDs
OCMG::
GetTerrainVIDs() {
  return m_terrainVIDs;
}

std::vector<std::pair<OCMG::State,OCMG::State>>
OCMG::
GetSingleObjectModeGraphEdgeTransitions(size_t _source, size_t _target, Robot* _object) {
  return m_omgEdgeTransitions[_object][std::make_pair(_source,_target)];
}

/*-------------------------------- Helpers -----------------------------------*/

void
OCMG::
ConstructObjectRoadmap(Robot* _object) {

  //TODO::Currently only samples start and goal positions.
  //      Update later to sample random points if desired.

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ConstructObjectRoadmap");

  // Initialize roadmap for object
  auto problem = this->GetMPProblem();
  auto group = problem->AddRobotGroup({_object},_object->GetLabel());
  m_solution->AddRobotGroup(group);
  m_groups.insert(group);
  m_objects.insert(_object);
  auto rm = m_solution->GetGroupRoadmap(group);

  // Add vertex for robot start
  auto startCfg = problem->GetInitialCfg(_object);
  GroupCfgType startGcfg(rm);
  startGcfg.SetRobotCfg(_object,std::move(startCfg));
  rm->AddVertex(startGcfg);

  // Add vertex for each robot goal
  auto coordinator = plan->GetCoordinator();
  auto decomp = problem->GetDecompositions(coordinator->GetRobot())[0].get();
  auto lib = this->GetMPLibrary();
  auto sm = lib->GetSampler(m_querySampler);
  lib->SetGroupTask(nullptr);

  for(auto st : decomp->GetGroupMotionTasks()) {
    auto gt = st->GetGroupMotionTask();
    if(gt->GetRobotGroup() != group)
      continue;

    // Should only have a single task
    for(auto iter = gt->begin(); iter != gt->end(); iter++) {

      // Sample goal configuration
      auto mt = *iter;
      lib->SetTask(&mt);
      auto boundary = iter->GetGoalConstraints()[0]->GetBoundary();

      std::vector<Cfg> samples;
      sm->Sample(1,m_maxQueryAttempts,boundary,std::back_inserter(samples));

      if(samples.empty())
        throw RunTimeException(WHERE) << "Unable to generate goal configuration for "
                                      << _object->GetLabel() << ".";
      
      lib->SetTask(nullptr);

      // Add to roadmap
      GroupCfgType gcfg(rm);
      gcfg.SetRobotCfg(_object,std::move(samples[0]));
      rm->AddVertex(gcfg);
    }
  }

}

void
OCMG::
ConstructRobotRoadmap(Robot* _robot) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ConstructRobotRoadmap");
  m_robots.insert(_robot);
  auto coordinator = plan->GetCoordinator();

  // TODO::Track associated motion conditions - for now, we can safely assume
  //       the open and closed constraints without and with objects.

  // Collect all formation conditions that could applu to this robot.
  std::set<FormationCondition*> formationConditions;

  auto as = this->GetTMPLibrary()->GetActionSpace();
  for(auto& interaction : as->GetActions()) {
    auto stages = interaction.second->GetStages();
    for(auto c : interaction.second->GetStageConditions(stages.front())) {
      // Check if this is a formation condition
      auto f = dynamic_cast<FormationCondition*>(as->GetCondition(c));
      if(!f)
        continue;

      // Check if the robot can be used here
      for(auto type : f->GetTypes()) {
        if(type == _robot->GetCapability()) {
          formationConditions.insert(f);
          break;
        }
      }
    }

    for(auto c : interaction.second->GetStageConditions(stages.back())) {
      // Check if this is a formation condition
      auto f = dynamic_cast<FormationCondition*>(as->GetCondition(c));
      if(!f)
        continue;

      // Check if the robot can be used here
      for(auto type : f->GetTypes()) {
        if(type == _robot->GetCapability()) {
          formationConditions.insert(f);
          break;
        }
      }
    }
  }

  // Convert to formations to plan for.
  auto problem = this->GetMPProblem();
  std::set<std::pair<RobotGroup*,Formation*>> formations;
  std::map<Formation*,FormationCondition*> formationMap;

  for(auto f : formationConditions) {

    // Check if it's just this robot by itself
    if(f->GetTypes().size() == 1) {
      auto group = problem->AddRobotGroup({_robot},_robot->GetLabel());
      formations.insert(std::make_pair(group,nullptr));
    }

    // Find a robot that matches the formation conditions
    std::vector<Robot*> robots = {_robot};
    std::string label = _robot->GetLabel();

    for(auto type : f->GetTypes()) {
      if(type == _robot->GetCapability())
        continue;

      bool found = false;
      for(auto& kv : coordinator->GetInitialRobotGroups()) {
        auto group = kv.first;
        for(auto robot : group->GetRobots()) {
          if(type != robot->GetCapability())
            continue;

          if(std::find(robots.begin(),robots.end(),robot) != robots.end())
            continue;

          found = true;
          robots.push_back(robot);
          label += ("::" + robot->GetLabel());
          break;          
        }

        if(found)
          break;
      }

      if(!found)
        throw RunTimeException(WHERE) << "Failed to build full group for " 
                                      << f->GetLabel()
                                      << ".";
    }

    // Construct group and formation
    auto group = problem->AddRobotGroup(robots,label);
    State state;
    state[group] = std::make_pair(nullptr,MAX_INT);

    std::cout << "ASSUMING ONLY A SINGLE ROLE MAP IS FEASIBLE." << std::endl;
    RoleMap roleMap;
    f->AssignRoles(roleMap,state);
    auto formation = f->GenerateFormation(roleMap);
    formationMap[formation] = f;
    formations.insert(std::make_pair(group,formation));
  }

  if(formations.empty())
    return;

  // Set all other robots to virtual
  for(auto& kv : coordinator->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(true);
    }
  }

  // Run roadmap strategy on all formations
  auto lib = this->GetMPLibrary();

  for(auto f : formations) {
    auto group = f.first;
    auto formation = f.second;

    // Configure formation in roadmap
    m_solution->AddRobotGroup(group);
    m_groups.insert(group);
    auto rm = m_solution->GetGroupRoadmap(group);
    rm->SetAllFormationsInactive();
    if(formation)
      rm->AddFormation(formation);
    else {
      // Add vertex for robot start
      auto startCfg = problem->GetInitialCfg(_robot);
      GroupCfgType startGcfg(rm);
      startGcfg.SetRobotCfg(_robot,std::move(startCfg));
      rm->AddVertex(startGcfg);
    }

    // Configure Group Task
    GroupTask gt(group);
    for(auto robot : group->GetRobots()) {
      MPTask mt(robot);
      
      // Add path constraint
      // Note, this is the hardcoding mentioned above instead of
      // of properly incorporating path constraints.
      if(!robot->GetMultiBody()->IsPassive()) {

        std::vector<std::pair<double,double>> values;
        for(size_t i = 0; i < robot->GetMultiBody()->DOF(); i++) {
          if(i == 1) {
            if(group->Size() == 1) {
              values.push_back(std::make_pair(0,0));
            }
            else {
              values.push_back(std::make_pair(.0001,.0001));
            }
          }
          else {
            values.push_back(std::make_pair(-1,1));
          }
        }

        auto bbx = std::unique_ptr<CSpaceBoundingBox>(
                     new CSpaceBoundingBox(robot->GetMultiBody()->DOF()));
        bbx->ResetBoundary(values,0.);
      }

      gt.AddTask(mt);
    }

    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(false);
    }

    // Run MPSolution to generate roadmap
    lib->Solve(problem,&gt,m_solution.get(),m_roadmapStrategy, LRand(), 
            this->GetNameAndLabel());

    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(true);
    }

    // Copy to all other duplicate pairs of robot - object
    if(group->Size() == 1)
      continue;

    // Iterate through all passive robots
    // TODO::Assuming each group is of size 2 and that only one is passive.
    auto passive = group->GetRobots()[0]->GetMultiBody()->IsPassive() ?
                   group->GetRobots()[0] :
                   group->GetRobots()[1];

    for(auto& kv : coordinator->GetInitialRobotGroups()) {
      auto group = kv.first;
      for(auto robot : group->GetRobots()) {
        if(!robot->GetMultiBody()->IsPassive() or robot == passive)
          continue;

        CopyRoadmap(rm,robot,formationMap[formation]);
      }
    }
  }

  // Set all other robots to virtual
  for(auto& kv : coordinator->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(false);
    }
  }

}

void
OCMG::
CopyRoadmap(GroupRoadmapType* _rm, Robot* _passive, FormationCondition* _condition) {

  auto originalGroup = _rm->GetGroup();
  auto problem = this->GetMPProblem();

  // Create new group
  std::vector<Robot*> robots = {_passive};
  std::string label = _passive->GetLabel();

  Robot* swapped = nullptr;

  for(auto robot : originalGroup->GetRobots()) {
    // Assuming only one robot of the passive type in the group
    if(robot->GetCapability() == _passive->GetCapability()) {
      swapped = robot;
      continue;
    }
    
    robots.push_back(robot);  
    label += ("::" + robot->GetLabel());
  }

  auto group = problem->AddRobotGroup(robots,label);
  m_solution->AddRobotGroup(group);
  m_groups.insert(group);
  auto rm = m_solution->GetGroupRoadmap(group);

  // Copy Formation
  State state;
  state[group] = std::make_pair(nullptr,MAX_INT);
  std::cout << "ASSUMING ONLY A SINGLE ROLE MAP IS FEASIBLE." << std::endl;
  RoleMap roleMap;
  _condition->AssignRoles(roleMap,state);
  auto formation = _condition->GenerateFormation(roleMap);

  rm->SetAllFormationsInactive();
  if(formation)
    rm->AddFormation(formation);

  // Copy Roadmap

  // Copy Vertices
  std::map<size_t,size_t> oldToNew;
  for(auto vit = _rm->begin(); vit != _rm->end(); vit++) {
    auto oldGcfg = vit->property();

    GroupCfgType newGcfg(rm);

    for(auto robot : originalGroup->GetRobots()) {
      Cfg cfg = oldGcfg.GetRobotCfg(robot);
      if(robot == swapped) {
        cfg.SetRobot(_passive);
      }

      newGcfg.SetRobotCfg(cfg.GetRobot(),std::move(cfg));
    }

    auto vid = rm->AddVertex(newGcfg);
    oldToNew[vit->descriptor()] = vid;
  }

  // Copy Edges
  for(auto vit = _rm->begin(); vit != _rm->end(); vit++) {
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      auto sourceGcfg = _rm->GetVertex(eit->source());
      auto targetGcfg = _rm->GetVertex(eit->target());

      auto source = oldToNew[eit->source()];
      auto target = oldToNew[eit->target()];

      // Copy over minimum info to query paths later
      auto property = eit->property();

      GroupLocalPlanType glp(rm);
      glp.SetWeight(property.GetWeight());
      glp.SetTimeSteps(property.GetTimeSteps());

      for(auto robot : originalGroup->GetRobots()) {
        auto edge = _rm->GetIndividualGraph(originalGroup->GetGroupIndex(robot))->GetEdge(
              sourceGcfg.GetVID(robot),targetGcfg.GetVID(robot));
        if(robot == swapped) {
          glp.SetEdge(_passive,std::move(edge));
        }
        else {
          glp.SetEdge(robot,std::move(edge));
        }
      }

      // Save edge
      rm->AddEdge(source,target,glp);
    }
  }
}

void
OCMG::
SampleInteractions() {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SampleInteractions");

  // Set all robots to virtual
  auto coordinator = plan->GetCoordinator();
  for(auto& kv : coordinator->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(false);
    }
  }

  auto as = this->GetTMPLibrary()->GetActionSpace();

  for(auto kv : as->GetActions()) {
    auto interaction = dynamic_cast<Interaction*>(kv.second);

    // Collect formation conditions
    std::vector<FormationCondition*> conditions;
    auto stages = interaction->GetStages();
    for(auto c : interaction->GetStageConditions(stages.front())) {
      auto f = dynamic_cast<FormationCondition*>(as->GetCondition(c));

      if(f)
        conditions.push_back(f);
    }

    // Collect all sets of matching groups to initiate interactions
    std::vector<std::map<FormationCondition*,RobotGroup*>> matchingGroups;
    matchingGroups.push_back({});
    
    for(auto f : conditions) {
      std::vector<std::map<FormationCondition*,RobotGroup*>> newMatches;

      for(auto group : m_groups) {

        // Check for match
        if(group->Size() != f->GetTypes().size())
          continue;

        bool match = true;

        std::set<Robot*> used;
        for(auto type : f->GetTypes()) {
          match = false;

          for(auto robot : group->GetRobots()) {
            if(used.count(robot))
              continue;

            if(type != robot->GetCapability())
              continue;

            used.insert(robot);
            match = true;
          }

          if(!match)
            break;
        }

        if(!match)
          continue;

        // If match, add to existing matching group sets
        for(auto map : matchingGroups) {

          // Check that group has not already been used
          bool conflict = false;
          for(auto kv : map) {
            for(auto r1 : kv.second->GetRobots()) {
              for(auto r2 : group->GetRobots()) {
                if(r1 == r2) {
                  conflict = true;
                  break;
                }
              }
              if(conflict)
                break;
            }
            if(conflict)
              break;
          }

          if(conflict)
            continue;

          // Add to set of new matches
          map[f] = group;
          newMatches.push_back(map);
        }
      }

      matchingGroups = newMatches;
    }

    // Sample interaction for each match
    for(auto map : matchingGroups) {
      State state;
      for(auto kv : map) {
        state[kv.second] = std::make_pair(nullptr,MAX_INT);
      }

      SampleInteraction(interaction,state);
    }
  }

  // Set all robots back to non-virtual
  for(auto& kv : coordinator->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(false);
    }
  }
}

bool
OCMG::
SampleInteraction(Interaction* _interaction, State _state) {

  // Set all relevant robots to non-virtual
  for(auto& kv : _state) {
    auto group = kv.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(false);
    }
  }

  // Check if state has a solo passive robot that needs to be sampled from start/goal
  RobotGroup* passive = nullptr;

  for(auto kv : _state) {
    auto group = kv.first;
    bool isPassive = true;
    for(auto robot : group->GetRobots()) {
      if(!robot->GetMultiBody()->IsPassive()) {
        isPassive = false;
        break;
      }
    }

    if(isPassive) {
      passive = group;
      break;
    }
  }

  if(passive) {
    return SampleInteractionWithPassive(_interaction, _state, passive);
  }

  bool success = false;
  for(size_t i = 0; i < m_interactionSamples; i++) {
    for(size_t j = 0; j < m_interactionAttempts; j++) {
      if(RunInteractionStrategy(_interaction,_state)) {
        success = true;
        break;
      }
    }
  }

  // Set all relevant robots back to virtual
  for(auto& kv : _state) {
    auto group = kv.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(true);
    }
  }

  return success;
}

bool
OCMG::
SampleInteractionWithPassive(Interaction* _interaction, State _state, RobotGroup* _passive) {

  // Sample an interaction from each of the passive objects already discovered vertices
  auto rm = m_solution->GetGroupRoadmap(_passive);
  bool success = false;

  auto pair = std::make_pair(rm,MAX_INT);

  for(auto vit = rm->begin(); vit != rm->end(); vit++) {
    auto vid = vit->descriptor();
    pair.second = vid;

    _state[_passive] = pair;

    success |= RunInteractionStrategy(_interaction,_state);
  }

  return success;
}

bool
OCMG::
RunInteractionStrategy(Interaction* _interaction, State _start) {
  auto is = this->GetInteractionStrategyMethod(
              _interaction->GetInteractionStrategyLabel());

  auto goalState = _start;

  if(!is->operator()(_interaction,goalState))
    return false;

  // Ensure that start is completely grounded
  for(auto kv : _start) {

    auto group = kv.first;
    auto oldRm = kv.second.first;
    // If the roadmap exists, move on
    if(oldRm)
      continue;

    GroupCfgType gcfg;
    auto stages = _interaction->GetStages();
    for(size_t i = 1; i < stages.size(); i++) {
      auto paths = _interaction->GetToStagePaths(stages[i]);
      if(paths.empty())
        continue;

      oldRm = _interaction->GetToStageSolution(stages[i])->GetGroupRoadmap(group);
        
      gcfg = GroupCfgType(oldRm);

      for(auto robot : group->GetRobots()) {
        for(const auto& path : paths) {
          if(path->GetRobot() == robot) {
            auto cfg = path->Cfgs().front();
            gcfg.SetRobotCfg(robot,std::move(cfg));
          }
        }
      }

      break;
    }

    auto vid = oldRm->AddVertex(gcfg);
    _start[group] = std::make_pair(oldRm,vid);
  }

  // Connect start and goal
  State copiedStart = CopyAndConnectState(_start);
  if(copiedStart.empty())
    return false;

  State copiedGoal = CopyAndConnectState(goalState);
  if(copiedGoal.empty())
    return false;

  // TODO::Save interaction paths
  // TODO::All we care about for planning SMART is that there is a transition, 
  //       don't necessarily need the path itself right now. 
 
  auto& savedInteractions = m_savedInteractions[_interaction];
  savedInteractions.emplace_back(copiedStart,copiedGoal);
 
  return true;
}

OCMG::State
OCMG::
CopyAndConnectState(State _state) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CopyAndConnectState");

  // Connect start and goals to existing roadmaps, if fails - quit
  auto lib = this->GetMPLibrary();
  auto connector = lib->GetConnector(m_connector);

  State copy;

  for(auto kv : _state) {

    // Copy over group cfg to internal roadmap
    auto group = kv.first;
    auto oldRm = kv.second.first;
    auto gcfg = oldRm->GetVertex(kv.second.second);

    auto rm = m_solution->GetGroupRoadmap(group);
    rm->SetAllFormationsInactive();
    for(auto f : gcfg.GetFormations()) {
      rm->AddFormation(f);
    }

    GroupCfgType newGcfg(rm);
    for(auto robot : group->GetRobots()) {
      auto cfg = gcfg.GetRobotCfg(robot);
      newGcfg.SetRobotCfg(robot,std::move(cfg));
    }

    auto vid = rm->AddVertex(newGcfg);
    copy[group] = std::make_pair(rm,vid);

    // Attempt to connect to roadmap
    connector->Connect(rm,vid);

    bool passive = true;
    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive())
        continue;

      passive = false;
      break;
    }

    if(passive)
      continue;

    if(rm->get_degree(vid) == 0) {
      if(m_debug) {
        std::cout << "Could not connect interaction to roadmap." << std::endl;
      }

      rm->DeleteVertex(vid);
      return {};
    }
  }

  if(m_debug) {
    std::cout << "Original state :" << std::endl;
    for(auto kv : _state) {
      auto group = kv.first;
      auto pair = kv.second;
      std::cout << group->GetLabel() << " : " << pair.first->GetVertex(pair.second) << std::endl;
    }
    std::cout << "Copied state. Now saved as :" << std::endl;
    for(auto kv : copy) {
      auto group = kv.first;
      auto pair = kv.second;
      std::cout << group->GetLabel() << " : " << pair.first->GetVertex(pair.second) << std::endl;
    }
  }

  return copy;
}

void
OCMG::
BuildIndividualObjectModeGraph() {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::BuildIndividualObjectModeGraph");

  MapTerrainVIDs();

  // Construct mode graph.
  auto coordinator = plan->GetCoordinator();
  m_omg = std::unique_ptr<SingleObjectModeGraph>(new SingleObjectModeGraph(coordinator->GetRobot()));
  
  // Add a vertex for every robot.
  std::vector<size_t> robotVIDs;
  for(auto robot : m_robots) {
    ModeInfo info(robot,nullptr,nullptr);
    auto vid = m_omg->AddVertex(info);
    robotVIDs.push_back(vid);
  }

  // Add vertex for every terrain.
  auto problem = this->GetMPProblem();
  auto env = problem->GetEnvironment();

  std::vector<size_t> terrainVIDs;
  for(auto& kv : env->GetTerrains()) {
    for(auto& terrain : kv.second) {
      ModeInfo info(nullptr,nullptr,&terrain);
      auto vid = m_omg->AddVertex(info);
      terrainVIDs.push_back(vid);
    }
  }

  // Add robot-robot edges
  for(size_t i = 0; i < robotVIDs.size()-1; i++) {
    auto vid1 = robotVIDs[i];
    auto robot1 = m_omg->GetVertex(vid1).robot;
    for(size_t j = i+1; j < robotVIDs.size(); j++) {
      auto vid2 = robotVIDs[j];
      auto robot2 = m_omg->GetVertex(vid2).robot;

      if(!IsReachable(robot1,robot2))
        continue;

      SaveEdgeTransitions(vid1,vid2);

      // Add edge - using atomic edges as indicated in paper
      double edge = 1.;
      m_omg->AddEdge(vid1,vid2,edge);
      m_omg->AddEdge(vid2,vid1,edge);
    }
  }

  // Add robot-terrain edges
  for(size_t i = 0; i < robotVIDs.size(); i++) {
    auto vid1 = robotVIDs[i];
    auto robot = m_omg->GetVertex(vid1).robot;
  
    for(size_t j = 0; j < m_terrainVIDs.size(); j++) {
      auto vid2 = terrainVIDs[j];
      auto terrain = m_omg->GetVertex(vid2).terrain;

      if(!IsReachable(terrain,robot))
        continue;

      SaveEdgeTransitions(vid1,vid2);

      // Add edge - using atomic edges as indicated in paper
      double edge = 1.;
      m_omg->AddEdge(vid1,vid2,edge);
      m_omg->AddEdge(vid2,vid1,edge);
    }
  }

  // Add self-edges
  for(size_t i = 0; i < robotVIDs.size(); i++) {
    auto vid = robotVIDs[i];
    m_omg->AddEdge(vid,vid,1.);
  }
  for(size_t i = 0; i < m_terrainVIDs.size(); i++) {
    auto vid = terrainVIDs[i];
    m_omg->AddEdge(vid,vid,1.);
  }

  if(m_debug) {
    std::cout << std::endl << "Printing Single Object Mode Graph" << std::endl;
    for(auto vit = m_omg->begin(); vit != m_omg->end(); vit++) {
      auto vid = vit->descriptor();
      auto vertex = vit->property();

      std::cout << vid << " : ";
      if(vertex.robot) {
        std::cout << vertex.robot->GetLabel() << std::endl;
      }
      else {
        for(auto& boundary : vertex.terrain->GetBoundaries()) {
          std::cout << "[";
          for(auto c : boundary->GetCenter()) {
            std::cout << c << ",";
          }
          std::cout << "], ";
        }
        std::cout << std::endl;
      }

      for(auto eit = vit->begin(); eit != vit->end(); eit++) {
        auto source = eit->source();
        auto target = eit->target();

        std::cout << "\t" << source << " -> " << target << std::endl;
      }
    } 
  }
}

void
OCMG::
MapTerrainVIDs() {
  // Map passive vids to terrains
  auto problem = this->GetMPProblem();
  auto env = problem->GetEnvironment();
  
  for(auto group : m_groups) {
    bool passive = true;
    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive())
        continue;

      passive = false;
      break;
    }

    if(!passive)
      continue;

    if(group->Size() > 1) 
      throw RunTimeException(WHERE) << "Only expecting passive groups of size one.";

    auto rm = m_solution->GetGroupRoadmap(group);

    for(auto vit = rm->begin(); vit != rm->end(); vit++) {
      auto vid = vit->descriptor();
      auto gcfg = vit->property();
      const size_t index = 0;
      auto cfg = gcfg.GetRobotCfg(index);

      for(auto& kv : env->GetTerrains()) {
        if(kv.first != cfg.GetRobot()->GetCapability())
          continue;

        bool found = false;

        for(auto& terrain : kv.second) {
          if(terrain.InTerrain(cfg)) {
            auto& map = m_terrainVIDs[&terrain];
            auto& set = map[rm];
            set.insert(vid);
            found = true;
            break;
          }
        }
    
        if(found)
          break;
      }
    }
  }

}

bool
OCMG::
IsReachable(const Terrain* _terrain, Robot* _robot) {

  auto mb = _robot->GetMultiBody();
  auto bbx = mb->GetBase()->GetWorldBoundingBox();

  auto boundary = _terrain->GetBoundary() ? _terrain->GetBoundary()
                                          : _terrain->GetBoundaries()[0].get();

  auto center1 = bbx.GetCentroid();
  auto center2 = boundary->GetCenter();

  double distance = 0;
  for(size_t i = 0; i < 3; i++) {
    distance += std::pow((center1[i] - center2[i]),2);
  }
  distance = std::sqrt(distance);

  // TODO::Compute accurately, cheating for ur5e because we know it's roughly one meter
  auto radius = mb->GetBoundingSphereRadius();
  radius = 1.02;

  auto maxDistFromCenter = boundary->GetMaxDist()/2;

  bool canReach = false;
  if(distance < radius - maxDistFromCenter)
    canReach = true;

  if(m_debug) {
    std::cout << _robot->GetLabel() << " can reach " << center2 
              << ": " << canReach << std::endl;
  }

  return canReach;
}

bool
OCMG::
IsReachable(Robot* _robot1, Robot* _robot2) {

  auto mb1 = _robot1->GetMultiBody();
  auto mb2 = _robot2->GetMultiBody();

  auto bbx1 = mb1->GetBase()->GetWorldBoundingBox();
  auto bbx2 = mb2->GetBase()->GetWorldBoundingBox();

  auto center1 = bbx1.GetCentroid();
  auto center2 = bbx2.GetCentroid();

  double distance = 0;
  for(size_t i = 0; i < 3; i++) {
    distance += std::pow(center1[i] - center2[i],2);
  }
  distance = std::sqrt(distance);

  auto radius1 = mb1->GetBoundingSphereRadius();
  auto radius2 = mb2->GetBoundingSphereRadius();

  // TODO::Compute accurately, cheating for ur5e because we know it's roughly one meter
  radius1 = 1;
  radius2 = 1;

  bool canReach = false;
  if(distance < radius1 + radius2)
    canReach = true;

  if(m_debug) {
    std::cout << _robot1->GetLabel() << " can reach " << _robot2->GetLabel() 
              << ": " << canReach << std::endl;
  }

  return canReach;
}

void
OCMG::
SaveEdgeTransitions(size_t _source, size_t _target) {
  // TODO::Assuming reversible edges
  
  auto problem = this->GetMPProblem();
  auto source = m_omg->GetVertex(_source);
  auto target = m_omg->GetVertex(_target);

  auto edgeKey = std::make_pair(_source,_target);
  auto reverseKey = std::make_pair(_target,_source);

  for(auto object : m_objects) {

    // Build source groups
    std::vector<GroupRoadmapType*> sourceGrms;

    {
      // Build objects starting state
      std::vector<Robot*> robots = {object};
      if(source.robot) {
        robots.push_back(source.robot);
      }
      auto group = problem->AddRobotGroup(robots,"");
      auto grm = m_solution->GetGroupRoadmap(group);
      sourceGrms.push_back(grm);

      // Build state of receiving robot
      if(target.robot) {
        robots = {target.robot};
        group = problem->AddRobotGroup(robots,"");
        grm = m_solution->GetGroupRoadmap(group);
        sourceGrms.push_back(grm); 
      }
    }

    // Build target groups
    std::vector<GroupRoadmapType*> targetGrms;

    {
      // Build objects ending state
      std::vector<Robot*> robots = {object};
      if(target.robot) {
        robots.push_back(target.robot);
      }
      auto group = problem->AddRobotGroup(robots,"");
      auto grm = m_solution->GetGroupRoadmap(group);
      targetGrms.push_back(grm);

      // Build state of delivering robot
      if(source.robot) {
        robots = {source.robot};
        group = problem->AddRobotGroup(robots,"");
        grm = m_solution->GetGroupRoadmap(group);
        targetGrms.push_back(grm); 
      }
    }

    // Identify matching transitions
    for(auto kv : m_savedInteractions) {
      for(auto transition : kv.second) {
        auto start = transition.first;
        auto goal = transition.second;

        bool startMatch = false;
        // Check if source matches start
        if(start.size() == sourceGrms.size()) {
          startMatch = true;

          for(auto grm : sourceGrms) {
            if(start.find(grm->GetGroup()) == start.end()) {
              startMatch = false;
              break;
            }
          }
        }

        // If the groups match, and the object is stationary, check that terrain matches state
        if(startMatch and source.terrain) {
          auto group = sourceGrms[0]->GetGroup();
          auto pair = start[group];
          auto gcfg = pair.first->GetVertex(pair.second);
          size_t index = 0;
          auto cfg = gcfg.GetRobotCfg(index);
          if(!source.terrain->InTerrain(cfg))
            startMatch = false;
        }
        
        // If not, check if target matches start
        bool reverse = false;
        if(!startMatch) {
          if(start.size() == targetGrms.size()) {
            reverse = true;

            for(auto grm : targetGrms) {
              if(start.find(grm->GetGroup()) == start.end()) {
                reverse = false;
                break;
              }
            }
          }
        }

        // If the groups match, and the object is stationary, check that terrain matches state
        if(reverse and target.terrain) {
          auto group = targetGrms[0]->GetGroup();
          auto pair = start[group];
          auto gcfg = pair.first->GetVertex(pair.second);
          size_t index = 0;
          auto cfg = gcfg.GetRobotCfg(index);
          if(!target.terrain->InTerrain(cfg))
            startMatch = true;
        }

        if(!reverse and !startMatch)
          continue;

        auto goalGrms = startMatch ? targetGrms : sourceGrms;

        // Check if goal grms match goal state 
        bool goalMatch = false;
        // Check if source matches start
        if(goal.size() == goalGrms.size()) {
          goalMatch = true;

          for(auto grm : goalGrms) {
            if(goal.find(grm->GetGroup()) == goal.end()) {
              goalMatch = false;
              break;
            }
          }
        }

        auto goalTerrain = startMatch ? target.terrain : source.terrain;
        // If the groups match, and the object is stationary, check that terrain matches state
        if(goalMatch and goalTerrain) {
          auto group = goalGrms[0]->GetGroup();
          auto pair = goal[group];
          auto gcfg = pair.first->GetVertex(pair.second);
          size_t index = 0;
          auto cfg = gcfg.GetRobotCfg(index);
          if(!goalTerrain->InTerrain(cfg))
            goalMatch = false;
        }

        if(!goalMatch)
          continue;

        auto sourceState = startMatch ? start : goal;
        auto targetState = startMatch ? goal : start;

        if(m_debug) {
          std::cout << "Saving interaction for " << object->GetLabel() 
                    << " from " << _source << " to " << _target << std::endl;
          std::cout << "Source State" << std::endl;
          for(auto kv : sourceState) {
            std::cout << kv.first->GetLabel() << " : " 
                      << kv.second.first->GetVertex(kv.second.second).PrettyPrint() 
                      << std::endl;
          }
          std::cout << "Target State" << std::endl;
          for(auto kv : targetState) {
            std::cout << kv.first->GetLabel() << " : " 
                      << kv.second.first->GetVertex(kv.second.second).PrettyPrint() 
                      << std::endl;
          }
        }

        auto& forward = m_omgEdgeTransitions[object][edgeKey]; 
        forward.emplace_back(sourceState,targetState);
        auto& backward = m_omgEdgeTransitions[object][reverseKey];
        backward.emplace_back(targetState,sourceState);
      }
    }

  }
}

/*----------------------------------------------------------------------------*/

std::ostream& 
operator<<(std::ostream& _os, const OCMG::ModeInfo) {
  return _os;
}

std::istream&
operator>>(std::istream& _is, const OCMG::ModeInfo) {
  return _is;
}
