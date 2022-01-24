#include "ObjectCentricModeGraph.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"

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
  auto startVID = m_graph.AddVertex(initialMode);
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

        // Add action to all non-conflicting existing edges
        for(auto combo : roleCombos) {
          for(auto pair : outgoing) {

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
              edge[robot] = std::make_pair(interaction,role.second);
            }

            outgoing.emplace_back(edge,used);
          }

          // Create single action edges for all combinations
          for(auto combo : roleCombos) {

            ObjectModeSwitch edge;
            std::set<Robot*> used;

            for(auto role : combo) {
              auto robot = role.first;
              edge[robot] = std::make_pair(interaction,role.second);
            }

            outgoing.emplace_back(edge,used);
          }
        }
      }

      // Add each outgoing edge and resulting vertex to graph
      for(auto pair : outgoing) {
        auto edge = pair.first;
        ApplyEdge(edge,mode);
      }
    }
  }
}

/*-------------------------------- Accessors ---------------------------------*/

const ObjectCentricModeGraph::GraphType*
ObjectCentricModeGraph::
GetGraph() const {
  return &m_graph;
}

/*----------------------------- Helper Functions -----------------------------*/

std::vector<std::vector<std::pair<Robot*,std::string>>>
ObjectCentricModeGraph::
GetAllApplications(Interaction* _interaction, VID _source) {

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

  // Collect all possible assignments for robots.
  std::unordered_map<std::string,std::vector<Robot*>> roleAssignments;

  for(auto condition : _interaction->GetStageConditions(0)) {

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
          bool conflict = false;
          for(auto p : partial) {
            if(p.first == robot) {
              conflict = true;
              break;
            }
          }
          if(conflict) 
            continue;

          partial.emplace_back(robot,role);
          extensions.push_back(partial);
        }
      }

      // No non-conflicting assignments exist
      if(extensions.empty())
        return {};

      partials = extensions;
    }
  }

  return partials;
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
ApplyEdge(ObjectModeSwitch _edge, VID _source) {


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

  // Collect remaingin robots by their interaction buddies
  std::unordered_map<Interaction*,std::unordered_map<std::string,Robot*>> interactionGroups;
  for(auto kv : _edge) {
    auto robot = kv.first;
    auto interaction = kv.second.first;
    auto role = kv.second.second;
    interactionGroups[interaction][role] = robot;
  }

  // Determine object assigmnent coming out of the interaction
  std::set<Robot*> placedObjects;
  for(auto kv : interactionGroups) {
    auto interaction = kv.first;
    auto roleMap = kv.second;

    auto stages = interaction->GetStages();
    auto conditions = interaction->GetStageConditions(stages.back());
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
          break;
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
    auto target = m_graph.AddVertex(newMode);
    m_graph.AddEdge(_source,target,_edge);
  }

}
/*----------------------------------------------------------------------------*/

