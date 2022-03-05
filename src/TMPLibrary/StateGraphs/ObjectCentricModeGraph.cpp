#include "ObjectCentricModeGraph.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/Constraints/BoundaryConstraint.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/Solution/Plan.h"

/*------------------------------ Construction --------------------------------*/
ObjectCentricModeGraph::
ObjectCentricModeGraph() {
  this->SetName("ObjectCentricModeGraph");
}

ObjectCentricModeGraph::
ObjectCentricModeGraph(XMLNode& _node) : StateGraph(_node) {
  this->SetName("ObjectCentricModeGraph");

  m_mpStrategy = _node.Read("mpStrategy",true,"","MPStrategy to build roadmaps");
  m_passiveStrategy = _node.Read("passiveStrategy",true,"","MPStrategy to sample passive object roadmaps");
}

ObjectCentricModeGraph::
~ObjectCentricModeGraph() { }

/*-------------------------------- Interface ---------------------------------*/

void
ObjectCentricModeGraph::
Initialize() {

  auto problem = this->GetMPProblem();

  // Initialize MPSolution
  auto c = this->GetPlan()->GetCoordinator();
  m_solution = std::unique_ptr<MPSolution>(new MPSolution(c->GetRobot()));

  // Construct initial state from coordinator
  State start;

  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    auto formation = kv.second;

    // Add individual robots to MPSolution
    for(auto& r : group->GetRobots()) {
      m_solution->AddRobot(r);
    }

    // Create new group roadmaps in MPSolution
    m_solution->AddRobotGroup(group);
    auto grm = m_solution->GetGroupRoadmap(group);

    // Add the initial formation to the roadmap and set it active
    if(formation) {
      grm->AddFormation(formation);
      grm->SetFormationActive(formation);
    }

    m_groups[group].insert(formation);

    // Create initial group cfgs
    auto gcfg = GroupCfg(grm);

    // Add initial cfg to individual roadmaps
    for(auto& r : group->GetRobots()) {
      auto rm = m_solution->GetRoadmap(r);
      auto cfg = problem->GetInitialCfg(r);
      auto vid = rm->AddVertex(cfg);

      // Update group cfg
      gcfg.SetRobotCfg(r,vid);
    }

    // Add group cfg to group roadmap
    auto vid = grm->AddVertex(gcfg);

    // Add group and vertex to start state
    start[group] = std::make_pair(grm,vid);
  }

  // Add goal states for each individual task
  auto decomp = problem->GetDecompositions(c->GetRobot())[0].get();
  for(auto task : decomp->GetGroupMotionTasks()) {
    auto gt = task->GetGroupMotionTask();
    auto group = gt->GetRobotGroup();
    m_solution->AddRobotGroup(group);
    auto rm = m_solution->GetGroupRoadmap(group);

    GroupCfg gcfg(rm);

    for(auto iter = gt->begin(); iter != gt->end(); iter++) {
      auto robot = iter->GetRobot();
      Cfg cfg(robot);
      auto constraint = iter->GetGoalConstraints()[0].get();
      auto boundary = constraint->GetBoundary();

      cfg.GetRandomCfg(boundary);

      gcfg.SetRobotCfg(robot,std::move(cfg));
    }

    rm->AddVertex(gcfg);
  }

  GenerateRepresentation(start);

  BuildRoadmaps();
}

void
ObjectCentricModeGraph::
GenerateRepresentation(const State& _start) {

  // Collect all robots
  m_robots.clear();
  for(auto kv : _start) {
    auto group = kv.first;
    for(auto robot : group->GetRobots()) {
      m_robots.push_back(robot);
    }
  }

  // Generate initial mode from input state 
  auto initialMode = GenerateInitialMode(_start); 

  // Build mode graph from actions and start state
  BuildModeGraph(initialMode);

  BuildSingleObjectModeGraph();

  if(m_debug)
    Print();
}

/*-------------------------------- Accessors ---------------------------------*/

ObjectCentricModeGraph::GraphType*
ObjectCentricModeGraph::
GetObjectModeGraph() {
  return &m_graph;
}
 
ObjectCentricModeGraph::SingleObjectModeGraph*
ObjectCentricModeGraph::
GetSingleObjectModeGraph() {
  return &m_singleModeGraph;
}

   
MPSolution* 
ObjectCentricModeGraph::
GetMPSolution() {
  return m_solution.get();
}

ObjectCentricModeGraph::GroupRoadmapType* 
ObjectCentricModeGraph::
GetGroupRoadmap(RobotGroup* _group) {
  auto rm = m_solution->GetGroupRoadmap(_group);
  if(rm)
    return rm;

  m_solution->AddRobotGroup(_group);
  return m_solution->GetGroupRoadmap(_group);
}

std::vector<Robot*>
ObjectCentricModeGraph::
GetObjects() {

  std::vector<Robot*> objects;

  for(auto r : m_robots) {
    if(r->GetMultiBody()->IsPassive()) {
      objects.push_back(r);
    }
  }
  
  return objects;
}
/*---------------------------------- Debug  ----------------------------------*/

void
ObjectCentricModeGraph::
Print() {
  std::cout << "Print Object Centric Mode Graph" << std::endl;

  for(auto vit = m_graph.begin(); vit != m_graph.end(); vit++) {

    auto vid = vit->descriptor();
    auto mode = vit->property();

    std::cout << "VID: " << vid << std::endl;
    for(auto kv : mode) {
      auto object = kv.first;
      auto state = kv.second;
      
      std::cout << "\t" << object->GetLabel();
      if(state.robot) {
        std::cout << " held by " << state.robot->GetLabel() << std::endl;
      }
      else {
        auto boundary = state.terrain->GetBoundary();

        if(!boundary) {
          boundary = state.terrain->GetBoundaries()[0].get();
        }

        std::cout << " in region [ ";
        for(size_t i = 0; i < boundary->GetDimension(); i++) {
          auto range = boundary->GetRange(i);
          std::cout << range.min << ":" << range.max;
          if(i+1 < boundary->GetDimension())
            std::cout << " ; ";
        } 
        std::cout << "]" << std::endl;
      }
    }
  }


  for(auto vit = m_graph.begin(); vit != m_graph.end(); vit++) {
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      auto source = eit->source();
      auto target = eit->target();
      auto edge = eit->property();

      std::cout << source << " -> " << target << std::endl;

      for(auto kv : edge) {
        auto interaction = kv.first;

        std::cout << "\t" << interaction->GetLabel() << std::endl;
  
        for(auto pair : kv.second) {
          auto reverse = pair.first;
          auto roleMap = pair.second;
    
          std::cout << "\t\tROLE MAP (reversed: " << reverse << ")" << std::endl;

          for(auto kv : roleMap) {
            std::cout << kv.first << " : " << kv.second->GetLabel() << std::endl;
          }
        }
      }

    }
  }
}

/*----------------------------- Helper Functions -----------------------------*/

void
ObjectCentricModeGraph::
BuildRoadmaps() {
  
  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();
  auto b = prob->GetEnvironment()->GetBoundary();

  for(auto kv : m_groups) {
    auto group = kv.first;
    m_solution->AddRobotGroup(group);
    auto rm = m_solution->GetGroupRoadmap(group);

    for(auto formation : kv.second) {

      // Configure roadmap
      rm->SetAllFormationsInactive();
      if(formation)
        rm->AddFormation(formation);

      bool passive = true;
      for(auto r : group->GetRobots()) {
        passive = passive and r->GetMultiBody()->IsPassive();
      }

      // Configure empty task for group
      GroupTask gt(group);
      for(auto robot : group->GetRobots()) {
        MPTask mt(robot);
        auto c = std::unique_ptr<BoundaryConstraint>(
            new BoundaryConstraint(robot,b->Clone()));
        mt.SetStartConstraint(std::move(c));
        gt.AddTask(mt);
      }

      std::string strategy = passive ? m_passiveStrategy
                                     : m_mpStrategy;

      // Call MPLibrary to build roadmap
      lib->Solve(prob,&gt,m_solution.get(),strategy,LRand(),
          this->GetNameAndLabel());
    }
  }
}

std::vector<std::vector<std::pair<Robot*,std::string>>>
ObjectCentricModeGraph::
GetAllApplications(Interaction* _interaction, VID _source, bool _reverse) {

  auto prob = this->GetMPProblem();
  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto mode = m_graph.GetVertex(_source);

  // Construct state from object centric mode
  State state;
  std::set<Robot*> used;

  // Grab all objects and any robots currently holding them
  for(auto kv : mode) {
  
    std::vector<Robot*> groupRobots;
    std::string groupLabel;

    auto object = kv.first;
    used.insert(object);
    groupRobots.push_back(object);
    groupLabel = object->GetLabel();

    auto robot = kv.second.robot;
    if(robot) {
      used.insert(robot);
      groupRobots.push_back(robot);
      groupLabel += ("::" + robot->GetLabel());
    }

    auto group = prob->AddRobotGroup(groupRobots,groupLabel);
    state[group] = std::make_pair(nullptr,MAX_INT);
  }

  // Grab all remaining robots
  for(auto robot : m_robots) {
    if(used.count(robot)) 
      continue;

    auto group = prob->AddRobotGroup({robot},robot->GetLabel());
    state[group] = std::make_pair(nullptr,MAX_INT);
  }

  // Collect group matches to conditions
  auto stages = _interaction->GetStages();
  auto stage = _reverse ? stages.back() : stages.front();
  std::unordered_map<FormationCondition*,std::vector<RobotGroup*>> groupMatches;

  for(auto condition : _interaction->GetStageConditions(stage)) {

    // Get formation condition
    auto f = dynamic_cast<FormationCondition*>(as->GetCondition(condition));

    if(!f)
      continue;

    auto roles = f->GetRoles();

    // Collect all matching groups
    std::vector<RobotGroup*> matches;
    
    for(auto kv : state) {
      // Check if group satisfies condition
      auto group = kv.first;

      if(group->Size() != roles.size())
        continue;

      std::set<Robot*> used;
      bool match = false;

      for(auto role : roles) {

        match = false;

        auto info = f->GetRoleInfo(role);
        auto type = info.type;

        for(auto robot : group->GetRobots()) {
          if(robot->GetCapability() == type) {
            match = true;
            used.insert(robot);
            break;
          }
        }

        if(!match)
          break;
      }

      if(!match)
        continue;

      matches.push_back(group);
    }

    groupMatches[f] = matches;
  }

  // Construct group matches into entire robot role assignments
  std::vector<std::vector<std::pair<Robot*,std::string>>> partials;

  for(auto kv : groupMatches) {
    auto f = kv.first;
    auto matches = kv.second;

    // Set of robot role assignments
    std::vector<std::vector<std::pair<Robot*,std::string>>> groupPartials;

    // Construct all permutations within individual group
    for(auto group : matches) {

      std::vector<std::vector<std::pair<Robot*,std::string>>> newPartials;

      for(auto role : f->GetRoles()) {
        auto info = f->GetRoleInfo(role);
        auto type = info.type;

        // Options for individual role
        std::vector<std::pair<Robot*,std::string>> roleOptions;


        for(auto robot : group->GetRobots()) {
          if(robot->GetCapability() == type) {
            if(groupPartials.empty()) {
              std::vector<std::pair<Robot*,std::string>> partial;
              partial.emplace_back(robot,role);
              newPartials.push_back(partial);
            }
            else {
              for(auto partial : groupPartials) {
                // Try to add assignment to existing partials

                bool conflict = false;
                for(auto assignment : partial) {
                  if(robot == assignment.first) {
                    conflict = true;
                    break;
                  }
                }

                // Quit if robot already exists in partial
                if(conflict) 
                  continue;

                partial.emplace_back(robot,role);
                newPartials.push_back(partial);
              }
            }
          }
        }
  
      }
  
      groupPartials = newPartials;

    }    


    if(groupPartials.empty())
      return {};

    std::vector<std::vector<std::pair<Robot*,std::string>>> candidatePartials = partials;
    for(auto roleOptions : groupPartials) {

      // Try to add group partials to system partials
      std::vector<std::vector<std::pair<Robot*,std::string>>> newPartials;

      for(auto gp : roleOptions) {  

        if(candidatePartials.empty()) {
          newPartials.push_back({gp});
          continue;
        }

        for(auto partial : candidatePartials) {
          // Check for conflcits
          bool conflict = false;

          //for(auto p1 : gp) {
            for(auto p2 : partial) {
              if(gp.first == p2.first) {
                conflict = true;
                break;
              }
            }
            if(conflict)
              break;
          //}

          if(conflict)
            continue;

          //for(auto pair : gp) {
            partial.push_back(gp);
          //}

          newPartials.push_back(partial);
        }
      }

      if(newPartials.empty())
        return {};
  
      candidatePartials = newPartials;
    }

    partials.clear();

    // Check if new additions obey formation constraints
    for(auto partial : candidatePartials) {
      RoleMap roleMap;
      size_t start = partial.size() - groupPartials.size();
      Robot* object;
      for(size_t i = start; i < partial.size(); i++)  {
        auto pair = partial[i];
        auto robot = pair.first;
        auto role = pair.second;
        roleMap[role] = robot;

        if(robot->GetMultiBody()->IsPassive())
          object = robot;
      }


      auto formation = f->GenerateFormation(roleMap);
      bool okay = formation == mode[object].formation;
      if(!okay and !formation)
        continue;
      okay = okay or (*formation == *(mode[object].formation));

      if(!okay)
        continue;

      partials.push_back(partial);
    }

  }

  
  for(auto partial : partials) {

  }

 
  return partials; 

  /*
  // Collect all possible assignments for robots.
  std::unordered_map<std::string,std::vector<Robot*>> roleAssignments;

  for(auto condition : _interaction->GetStageConditions(stages[0])) {

    // Get formation condition
    auto f = dynamic_cast<FormationCondition*>(as->GetCondition(condition));

    if(!f)
      continue;

    // Collect all possible assignments of robots to types/roles
    auto roles = f->GetRoles();

    for(auto role : roles) {

      auto info = f->GetRoleInfo(role);
      auto type = info.type;

      for(auto robot : m_robots) {
        if(robot->GetCapability() == type) {
          roleAssignments[role].push_back(robot);
        }
      }

      // Check that condition can be satisfied
      if(roleAssignments[role].empty())
        return {};
    }
  }
  
  std::vector<std::vector<std::pair<Robot*,std::string>>> partials;

  for(auto kv : roleAssignments) {
    auto role = kv.first;
    auto options = kv.second;

    // If partials is empty, initialize with this role's assignments
    if(partials.empty()) {
      for(auto robot : options) {
        auto p = std::make_pair(robot,role);
        partials.push_back({p});
      }
    }
    // Create non-conflciting extensions
    else {
      std::vector<std::vector<std::pair<Robot*,std::string>>> extensions;

      for(auto partial : partials) {
        for(auto robot : options) {

          auto extended = partial;

          bool conflict = false;
          for(auto p : partial) {
            if(p.first == robot) {
              conflict = true;
              break;
            }
          }
          if(conflict) 
            continue;

          extended.emplace_back(robot,role);
          extensions.push_back(extended);
        }
      }

      // No non-conflicting assignments exist
      if(extensions.empty())
        return {};

      partials = extensions;
    }
  }

  return partials;
  */
}


ObjectCentricModeGraph::ObjectMode
ObjectCentricModeGraph::
GenerateInitialMode(const State& _start) {

  ObjectMode initialMode;

  const auto& terrainMap = this->GetMPProblem()->GetEnvironment()->GetTerrains();

  // Go though groups in _start and determine current mode of objects
  for(auto kv : _start) {

    // Separate group into objects and robots
    auto group = kv.first;
    auto grm = kv.second.first;
    std::vector<Robot*> objects;
    std::vector<Robot*> robots;

    for(auto r : group->GetRobots()) {
      if(r->GetMultiBody()->IsPassive()) {
        objects.push_back(r);
      }
      else {
        robots.push_back(r);
      }
    }

    // Continue if no objects are present    
    if(objects.empty())
      continue;

    // Check if object is in a group with a robot, assume this implies grasp
    if(objects.size() < group->Size()) {
      // For now, assume (first and only) object is held by one robot
      auto object = objects[0];
      auto robot = robots[0];
      auto formations = grm->GetActiveFormations();
      if(formations.size() != 1)
        throw RunTimeException(WHERE) << "Not able to parse initial formation counts other than 1.\n"
                                      << "Current formation count is " << formations.size();
      auto formation = formations.begin();
      initialMode[object] = ModeInfo(robot,*formation,nullptr);
    }
    // Otherwise, collect stable grasp region it lies within
    else {
      auto object = objects[0];
      auto vid = kv.second.second;
      auto gcfg = grm->GetVertex(vid);
      auto cfg = gcfg.GetRobotCfg(object);
      
      // Iterate through boundaries and identify region
      bool contained = false;
      for(const auto& terrain : terrainMap.at(object->GetCapability())) {
        if(!terrain.InTerrain(cfg)) {
          continue;
        }

        contained = true;
     
        initialMode[object] = ModeInfo(nullptr,nullptr,&terrain);
        break;
      }
      
      if(!contained) 
        throw RunTimeException(WHERE) << "Object " << object->GetLabel() 
                                      << " is not contained within any known region.";
    }
  }

  return initialMode;
}

void
ObjectCentricModeGraph::
BuildModeGraph(ObjectMode& _initialMode) {

  auto startVID = m_graph.AddVertex(_initialMode);
  std::set<VID> newModes = {startVID};

  auto as = this->GetTMPLibrary()->GetActionSpace();

  std::unordered_map<Interaction*,std::set<VID>> expanded;

  while(!newModes.empty()) {
    auto toExpand = newModes;
    newModes.clear();

    for(auto mode : toExpand) {

      // Pair is outgoing edge and robots used
      std::vector<std::pair<ObjectModeSwitch,std::set<Robot*>>> outgoing;

      for(auto pair : as->GetActions()) {

        // Collect all applications of interaction to mode        
        auto interaction = static_cast<Interaction*>(pair.second);

        auto roleCombos = GetAllApplications(interaction, mode);
        ExpandApplications(roleCombos,outgoing,interaction);

        if(interaction->IsReversible()) {
          auto roleCombos = GetAllApplications(interaction, mode,true);
          ExpandApplications(roleCombos,outgoing,interaction,true);
        }
      
      }

      // Add each outgoing edge and resulting vertex to graph
      for(auto pair : outgoing) {
        auto edge = pair.first;
        auto used = pair.second;
        ApplyEdge(edge,mode,newModes,used);
      }
    }
  }
}

void
ObjectCentricModeGraph::
ExpandApplications(const std::vector<std::vector<std::pair<Robot*,std::string>>>& _roleCombos, 
                   std::vector<std::pair<ObjectModeSwitch,std::set<Robot*>>>& _outgoing, 
                   Interaction* _interaction, bool _reverse, bool _initial) {
       
  if(!_initial and _outgoing.empty()) {
    return;
  }

  std::vector<std::pair<ObjectModeSwitch,std::set<Robot*>>> newOutgoing;
 
  // Add action to all non-conflicting existing edges
  for(auto combo : _roleCombos) {
    for(auto pair : _outgoing) {

      // Check if existing edge conflict with new action
      auto used = pair.second;
      bool conflict = false;

      for(auto role : combo) {
        auto robot = role.first;
        if(used.count(robot)) {
          conflict = true;
          break;
        }
      }

      if(conflict) {
        continue;
      }

      // Copy edge and add new action to it
      auto edge = pair.first;
      RoleMap roleMap;

      for(auto role : combo) {
        auto robot = role.first;
        auto label = role.second;
        roleMap[label] = robot;
        used.insert(robot);
      }
    
      edge[_interaction].emplace_back(_reverse,roleMap);

      newOutgoing.emplace_back(edge,used);
    }
  }
  
  if(_initial) {
    // Create single action edges for all combinations
    for(auto combo : _roleCombos) {

      ObjectModeSwitch edge;
      std::set<Robot*> used;
      RoleMap roleMap;

      for(auto role : combo) {
        auto robot = role.first;
        auto label = role.second;
        roleMap[label] = robot;
        used.insert(robot);
      }

      edge[_interaction].emplace_back(_reverse,roleMap);

      newOutgoing.emplace_back(edge,used);
    }
  }

  ExpandApplications(_roleCombos, newOutgoing, _interaction, _reverse, false);

  for(auto elem : newOutgoing) {
    _outgoing.push_back(elem);
  }
}

void
ObjectCentricModeGraph::
ApplyEdge(ObjectModeSwitch _edge, VID _source, 
          std::set<VID>& _newModes, const std::set<Robot*>& _used) {


  auto prob = this->GetMPProblem();
  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto mode = m_graph.GetVertex(_source);
  const auto& terrainMap = this->GetMPProblem()->GetEnvironment()->GetTerrains();

  ObjectMode newMode;

  // Copy over static objects
  for(auto object : m_robots) {
    if(!object->GetMultiBody()->IsPassive())
      continue;
    
    if(!_used.count(object)) 
      newMode[object] = mode[object];
  }

  // Determine object assigmnent coming out of the interaction
  std::set<Robot*> placedObjects;
  for(auto kv : _edge) {
    auto interaction = kv.first;

    for(auto pair : kv.second) {
      auto reverse = pair.first;
      auto roleMap = pair.second;

      auto stages = interaction->GetStages();
      auto stage = reverse ? stages.front() : stages.back();
      auto conditions = interaction->GetStageConditions(stage);

      for(auto c : conditions) {
        auto f = dynamic_cast<FormationCondition*>(as->GetCondition(c));
        if(!f)
          continue;

        // Check if condition contains object
        Robot* object = nullptr;
        Robot* robot = nullptr;
        RoleMap temp;
        for(auto role : f->GetRoles()) {
          auto r = roleMap[role];
          if(r->GetMultiBody()->IsPassive()) {
            object = r;
          }
          else {
            robot = r;
          }
            
          temp[role] = r;
        }

        if(!object)
          continue;

        Formation* formation = nullptr;

        if(robot) {

          // Check if robot can reach region or robot
          auto oldModeInfo = mode[object];
          if(oldModeInfo.robot and !IsReachable(oldModeInfo.robot,robot)) {
            return;
          }
          else if(oldModeInfo.terrain and !IsReachable(mode[object].terrain,robot)) {
            return;
          }

          formation = f->GenerateFormation(temp);
          newMode[object] = ModeInfo(robot,formation,nullptr);
        }
        else {
          placedObjects.insert(object);
        }

        // Add robot group defining this formation saved set of groups
        std::vector<Robot*> robots;
        std::string label = "";
        for(auto kv : temp) {
          auto r = kv.second;
          robots.push_back(r);
          label += ("::" + r->GetLabel());
        }
  
        auto group = prob->AddRobotGroup(robots,label);
        m_groups[group].insert(formation);
      }
    }
  }

  //TODO::Check reachability on this instead of assuming all robots can reach all regions
  //TODO::Check that capacity constraints aren't violated in placement

  std::vector<ObjectMode> partials = {newMode};

  for(auto object : placedObjects) {

    std::vector<ObjectMode> newPartials;

    for(auto partial : partials) {
      for(const auto& terrain : terrainMap.at(object->GetCapability())) {
        // If reachable and terrain has capacity
        if(!IsReachable(&terrain,mode[object].robot))
          continue;
        partial[object] = ModeInfo(nullptr,nullptr,&terrain);
        newPartials.push_back(partial);
      }
    }

    partials = newPartials;
  }

  for(auto newMode : partials) {
    auto size = m_graph.Size();
    auto target = m_graph.AddVertex(newMode);
    m_graph.AddEdge(_source,target,_edge);
    if(target == size)
      _newModes.insert(target);
  }

}
    
void
ObjectCentricModeGraph::
BuildSingleObjectModeGraph() {

  // Add all robots to graph
  std::map<Robot*,size_t> robotVIDs;

  for(auto robot : m_robots) {

    if(robot->GetMultiBody()->IsPassive())
      continue;

    ModeInfo info(robot);
    auto vid = m_singleModeGraph.AddVertex(info);
    robotVIDs[robot] = vid;
  }

  // Add all terrain regions to graph
  const auto& terrainMap = this->GetMPProblem()->GetEnvironment()->GetTerrains();
  std::map<const Terrain*,size_t> regionVIDs;

  for(auto& kv : terrainMap) {
    for(const auto& terrain : kv.second) {

      ModeInfo info(nullptr,nullptr,&terrain);
      auto vid = m_singleModeGraph.AddVertex(info);
      regionVIDs[&terrain] = vid;
    }
  }

  // Add robot - robot edges
  for(auto iter1 = robotVIDs.begin(); iter1 != robotVIDs.end(); iter1++) {
    for(auto iter2 = iter1; iter2 != robotVIDs.end(); iter2++) {

      auto vid1 = iter1->second;
      auto vid2 = iter2->second;

      if(vid1 == vid2) {
        m_singleModeGraph.AddEdge(vid1,vid2,0);
        continue;
      }
     
      auto robot1 = iter1->first;
      auto robot2 = iter2->first;

      if(!IsReachable(robot1,robot2))
        continue;

      auto mb1 = robot1->GetMultiBody();
      auto mb2 = robot2->GetMultiBody();

      auto bbx1 = mb1->GetBase()->GetWorldBoundingBox();
      auto bbx2 = mb2->GetBase()->GetWorldBoundingBox();

      auto center1 = bbx1.GetCentroid();
      auto center2 = bbx2.GetCentroid();

      double distance = 0;
      for(size_t i = 0; i < 3; i++) {
        distance += std::pow((center1[i] - center2[i]),2);
      }
      distance = std::sqrt(distance);

      //THIS IS RESETING IT TO ACTION DISTANCE
      distance = 1.0;
 
      m_singleModeGraph.AddEdge(vid1,vid2,distance);
      m_singleModeGraph.AddEdge(vid2,vid1,distance);
    }
  }

  // Add robot - region edges
  for(auto iter1 = robotVIDs.begin(); iter1 != robotVIDs.end(); iter1++) {
    for(auto iter2 = regionVIDs.begin(); iter2 != regionVIDs.end(); iter2++) {

      auto robot = iter1->first;
      auto terrain = iter2->first;

      if(!IsReachable(terrain,robot))
        continue;

      auto vid1 = iter1->second;
      auto vid2 = iter2->second;

      auto mb = robot->GetMultiBody();
      auto bbx = mb->GetBase()->GetWorldBoundingBox();

      auto boundary = terrain->GetBoundary() ? terrain->GetBoundary()
                                             : terrain->GetBoundaries()[0].get();


      auto center1 = bbx.GetCentroid();
      auto center2 = boundary->GetCenter();

      double distance = 0;
      for(size_t i = 0; i < 3; i++) {
        distance += std::pow((center1[i] - center2[i]),2);
      }
      distance = std::sqrt(distance);

      //THIS IS RESETING IT TO ACTION DISTANCE
      distance = 1.0;
 
      m_singleModeGraph.AddEdge(vid1,vid2,distance);
      m_singleModeGraph.AddEdge(vid2,vid1,distance);
    }
  }

  // Add region self edges
  for(auto iter = regionVIDs.begin(); iter != regionVIDs.end(); iter++) {
    auto vid = iter->second;
    m_singleModeGraph.AddEdge(vid,vid,0);
  }
 
}

bool
ObjectCentricModeGraph::
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
ObjectCentricModeGraph::
IsReachable(Robot* _robot1, Robot* _robot2) {

  auto mb1 = _robot1->GetMultiBody();
  auto mb2 = _robot2->GetMultiBody();

  auto bbx1 = mb1->GetBase()->GetWorldBoundingBox();
  auto bbx2 = mb2->GetBase()->GetWorldBoundingBox();

  auto center1 = bbx1.GetCentroid();
  auto center2 = bbx2.GetCentroid();

  double distance = 0;
  for(size_t i = 0; i < 3; i++) {
    distance += center1[i] * center2[i];
  }
  distance = std::sqrt(distance);

  auto radius1 = mb1->GetBoundingSphereRadius();
  auto radius2 = mb2->GetBoundingSphereRadius();

  // TODO::Compute accurately, cheating for ur5e because we know it's roughly one meter
  radius1 = 1;
  radius2 = 1;

  if(distance < radius1 + radius2)
    return true;

  return false;
}

/*----------------------------------------------------------------------------*/

std::ostream& 
operator<<(std::ostream& _os, const ObjectCentricModeGraph::ObjectMode) {
  return _os;
}

std::istream&
operator>>(std::istream& _is, const ObjectCentricModeGraph::ObjectMode) {
  return _is;
}

std::ostream& 
operator<<(std::ostream& _os, const ObjectCentricModeGraph::ModeInfo) {
  return _os;
}

std::istream&
operator>>(std::istream& _is, const ObjectCentricModeGraph::ModeInfo) {
  return _is;
}

