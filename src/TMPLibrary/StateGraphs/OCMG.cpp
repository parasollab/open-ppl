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
  GroupCfg startGcfg(rm);
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
      GroupCfg gcfg(rm);
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
  auto coordinator = plan->GetCoordinator();
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

    RoleMap roleMap;
    f->AssignRoles(roleMap,state);
    auto formation = f->GenerateFormation(roleMap);
    formationMap[formation] = f;
    formations.insert(std::make_pair(group,formation));
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

    // Run MPSolution to generate roadmap
    lib->Solve(problem,&gt,m_solution.get(),m_roadmapStrategy, LRand(), 
            this->GetNameAndLabel());

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

    GroupCfg newGcfg(rm);

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
        auto edge = _rm->GetRoadmap(originalGroup->GetGroupIndex(robot))->GetEdge(
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
}

bool
OCMG::
SampleInteraction(Interaction* _interaction, State _state) {

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

    oldRm = m_solution->GetGroupRoadmap(group);
    GroupCfg gcfg(oldRm);
    gcfg = GroupCfg(oldRm);
    auto stages = _interaction->GetStages();
    for(size_t i = 1; i < stages.size(); i++) {
      auto paths = _interaction->GetToStagePaths(stages[i]);
      if(paths.empty())
        continue;

      for(auto robot : group->GetRobots()) {
        for(const auto& path : paths) {
          if(path->GetRobot() == robot) {
            auto cfg = path->Cfgs().front();
            gcfg.SetRobotCfg(robot,std::move(cfg));
          }
        }
      }
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

    GroupCfg newGcfg(rm);
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
  radius = 1;

  auto maxDistFromCenter = boundary->GetMaxDist()/2;

  if(distance < radius - maxDistFromCenter)
    return true;

  return false;
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
  radius1 = .9;
  radius2 = .9;

  if(distance < radius1 + radius2)
    return true;

  return false;
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
