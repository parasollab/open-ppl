#include "ObjectCentricModeGraph.h"

#include "Behaviors/Agents/Coordinator.h"

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

  GenerateRepresentation(start);
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

  if(m_debug)
    Print();
}

/*-------------------------------- Accessors ---------------------------------*/

const ObjectCentricModeGraph::GraphType*
ObjectCentricModeGraph::
GetGraph() const {
  return &m_graph;
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
      if(state.first) {
        std::cout << " held by " << state.first->GetLabel() << std::endl;
      }
      else {
        auto boundary = state.second->GetBoundary();

        if(!boundary) {
          boundary = state.second->GetBoundaries()[0].get();
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
        auto robot = kv.first;
        auto action = kv.second;
        if(action.first.first) {
          std::cout << "\t"   << robot->GetLabel() 
                    << " in " << action.first.first->GetLabel() 
                    << " as " << action.second 
                    << std::endl;
        }
        else {
          std::cout << "\t" << robot->GetLabel() 
                    << " does not perform an action" 
                    << std::endl;
        }
      }
    }
  }
}

/*----------------------------- Helper Functions -----------------------------*/

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

    auto robot = kv.second.first;
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

    for(auto roleOptions : groupPartials) {

      // Try to add group partials to system partials
      std::vector<std::vector<std::pair<Robot*,std::string>>> newPartials;

      for(auto gp : roleOptions) { 

        if(partials.empty()) {
          newPartials.push_back({gp});
          continue;
        }

        for(auto partial : partials) {
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
      partials = newPartials;
    }
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
      initialMode[object] = std::make_pair(robot,nullptr);
    }
    // Otherwise, collect stable grasp region it lies within
    else {
      auto object = objects[0];
      auto grm = kv.second.first;
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
     
        Robot* empty = nullptr;
 
        initialMode[object] = std::make_pair(empty,&terrain);
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
        ApplyEdge(edge,mode,newModes);
      }
    }
  }
}

void
ObjectCentricModeGraph::
ExpandApplications(const std::vector<std::vector<std::pair<Robot*,std::string>>>& _roleCombos, 
                   std::vector<std::pair<ObjectModeSwitch,std::set<Robot*>>>& _outgoing, 
                   Interaction* _interaction, bool _reverse) {
        
  auto inter = std::make_pair(_interaction,_reverse);

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
      for(auto role : combo) {
        auto robot = role.first;
        edge[robot] = std::make_pair(inter,role.second);
      }

      _outgoing.emplace_back(edge,used);
    }
  }
  // Create single action edges for all combinations
  for(auto combo : _roleCombos) {

    ObjectModeSwitch edge;
    std::set<Robot*> used;

    for(auto role : combo) {
      auto robot = role.first;
      edge[robot] = std::make_pair(inter,role.second);
      used.insert(robot);
    }

    _outgoing.emplace_back(edge,used);
  }
}

void
ObjectCentricModeGraph::
ApplyEdge(ObjectModeSwitch _edge, VID _source, std::set<VID>& _newModes) {


  auto as = this->GetTMPLibrary()->GetActionSpace();
  auto mode = m_graph.GetVertex(_source);
  const auto& terrainMap = this->GetMPProblem()->GetEnvironment()->GetTerrains();

  ObjectMode newMode;

  // Copy over static objects
  for(auto kv : mode) {
    auto object = kv.first;
    
    auto iter = _edge.find(object);
    if(iter == _edge.end()) 
      newMode[object] = mode[object];
  }

  // Collect remaining robots by their interaction buddies
  std::unordered_map<Interaction*,std::pair<bool,std::unordered_map<std::string,Robot*>>> interactionGroups;
  for(auto kv : _edge) {
    auto robot = kv.first;
    auto interaction = kv.second.first.first;
    auto reverse = kv.second.first.second;
    auto role = kv.second.second;
    interactionGroups[interaction].first = reverse;
    interactionGroups[interaction].second[role] = robot;
  }

  // Determine object assigmnent coming out of the interaction
  std::set<Robot*> placedObjects;
  for(auto kv : interactionGroups) {
    auto interaction = kv.first;
    auto reverse = kv.second.first;
    auto roleMap = kv.second.second;

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
      for(auto role : f->GetRoles()) {
        auto r = roleMap[role];
        if(r->GetMultiBody()->IsPassive()) {
          object = r;
        }
        robot = r;
      }

      if(!object)
        continue;

      if(robot) {
        newMode[object] = std::make_pair(robot,nullptr);
      }
      else {
        placedObjects.insert(object);
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
        partial[object] = std::make_pair(nullptr,&terrain);
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
/*----------------------------------------------------------------------------*/

