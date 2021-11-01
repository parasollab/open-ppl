#include "ModeGraph.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/MPProblem.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"
#include "TMPLibrary/InteractionStrategies/InteractionStrategyMethod.h"
#include "TMPLibrary/Solution/Plan.h"

#include <set>

/*------------------------------ Construction --------------------------------*/

ModeGraph::
ModeGraph() {
  this->SetName("ModeGraph");
}

ModeGraph::
ModeGraph(XMLNode& _node) : StateGraph(_node) {
  this->SetName("ModeGraph");

  m_expansionStrategy = _node.Read("expansionStrategy",true,"",
                      "MPStrategy label to build initial roadaps.");
  m_queryStrategy = _node.Read("queryStrategy",true,"",
                      "MPStrategy label to query roadaps.");
  m_numSamples = _node.Read("numSamples",false,1,1,1000,
                      "The number of samples to generate for each transtion.");
  m_maxAttempts = _node.Read("maxAttempts",false,1,1,1000,
                      "The max number of attempts to generate a sample.");
}

ModeGraph::
~ModeGraph() {}

/*-------------------------------- Interface ---------------------------------*/

void
ModeGraph::
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

    // Add the initial formatino to the roadmap and set it active
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
ModeGraph::
GenerateRepresentation(const State& _start) {

  GenerateModeHypergraph(_start);
  SampleTransitions();
  GenerateRoadmaps(_start);
  ConnectTransitions();

}

/*-------------------------------- Accessors ---------------------------------*/

ModeGraph::ModeHypergraph&
ModeGraph::
GetModeHypergraph() {
  return m_modeHypergraph;
}

ModeGraph::GroundedHypergraph&
ModeGraph::
GetGroundedHypergraph() {
  return m_groundedHypergraph;
}

ModeGraph::GroupRoadmapType*
ModeGraph::
GetGroupRoadmap(RobotGroup* _group) {
  return m_solution->GetGroupRoadmap(_group);
}

/*---------------------------- Helper Functions ------------------------------*/

void
ModeGraph::
GenerateModeHypergraph(const State& _start) {
  auto as = this->GetTMPLibrary()->GetActionSpace();

  // Extract initial submodes
  std::vector<VID> newModes;
  for(auto kv : _start) {
    std::unique_ptr<Mode> mode = std::unique_ptr<Mode>(new Mode);
    mode->robotGroup = kv.first;
    auto rm = kv.second.first;
    mode->formations = rm->GetActiveFormations();
    auto tasks = this->GetMPProblem()->GetTasks(mode->robotGroup);

    for(auto task : tasks) {

      for(auto individualTask : *(task.get())) {

        auto& constraints = individualTask.GetPathConstraints();

        for(auto& constraint : constraints) {

          auto c = constraint->Clone();

          mode->constraints.push_back(std::move(c));
        }
      }
    }

    m_modes.push_back(std::move(mode));

    auto vid = m_modeHypergraph.AddVertex(m_modes.back().get());
    newModes.push_back(vid);
  }

  // Keep track of already expanded mode/action combinations
  std::unordered_map<Action*,std::set<std::vector<VID>>> appliedActions;

  do {
    
    // Add new modes to the hyerpgraph
    for(auto vid : newModes) {
      m_modeGroundedVertices[vid] = std::unordered_set<VID>();
    }

    // Clear added modes
    newModes.clear();

    // Apply actions to discovered modes to make new modes
    for(auto actionLabel : as->GetActions()) {
      auto action = actionLabel.second;
      ApplyAction(action,appliedActions[action],newModes);
    }

  } while(!newModes.empty());
}

void
ModeGraph::
SampleTransitions() {

  // For each edge in the mode graph, generate n samples
  for(auto& kv : m_modeHypergraph.GetHyperarcMap()) {

    auto& hyperarc = kv.second;
    auto interaction = dynamic_cast<Interaction*>(hyperarc.property);

    State modeSet;
    std::unordered_map<RobotGroup*,Mode*> tailModeMap;
    for(auto vid : hyperarc.tail) {
      auto mode = m_modeHypergraph.GetVertex(vid).property;
      modeSet[mode->robotGroup] = std::make_pair(nullptr,MAX_INT);
      tailModeMap[mode->robotGroup] = mode;
    }

    auto label = interaction->GetInteractionStrategyLabel();
    auto is = this->GetInteractionStrategyMethod(label);

    for(size_t i = 0; i < m_numSamples; i++) {

      for(size_t j = 0; j < m_maxAttempts; j++) {

        // Make state copy to pass by ref and get output state
        auto goalSet = modeSet;

        if(!is->operator()(interaction,goalSet))
          continue;

        // Save interaction paths
        SaveInteractionPaths(interaction,modeSet,goalSet,tailModeMap);
        break;
      }
    }
  }
}

void
ModeGraph::
GenerateRoadmaps(const State& _start) {

  // If mode is initial mode, add starting vertex
  for(const auto& kv : _start) {
    auto group = kv.first;
    auto grm = kv.second.first;
    auto vid = kv.second.second;
    auto gcfg = grm->GetVertex(vid);

    auto newGrm = m_solution->GetGroupRoadmap(group);
    auto newGcfg = gcfg.SetGroupRoadmap(newGrm);
    newGrm->AddVertex(newGcfg);
  }

  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();
 
  // For each mode in the mode hypergraph, run the expansion strategy
  for(auto kv : m_modeHypergraph.GetVertexMap()) {
    auto vertex = kv.second;
    auto mode = vertex.property;

    // Initialize dummy task
    auto task = new GroupTask(mode->robotGroup);

    for(auto r : mode->robotGroup->GetRobots()) {
      auto t = MPTask(r);
      task->AddTask(t);
    } 

    // Call the MPLibrary solve function to expand the roadmap
    lib->SetPreserveHooks(true);
    lib->Solve(prob,task,m_solution.get(),m_expansionStrategy, LRand(), 
            "ExpandModeRoadmap");
    lib->SetPreserveHooks(false);

    delete task;
  }
}

void
ModeGraph::
ConnectTransitions() {
  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();

  // For each mode in the mode hypergraph, attempt to connect grounded transition samples
  for(auto kv1 : m_modeHypergraph.GetVertexMap()) {
  
    for(auto vid1 : m_modeGroundedVertices[kv1.first]) {

      auto vertex1 = m_groundedHypergraph.GetVertex(vid1);

      // Create start constraint from vertex
      auto startGcfg = vertex1.property.first->GetVertex(vertex1.property.second);
      std::vector<CSpaceConstraint> startConstraints;

      auto grm = startGcfg.GetGroupRoadmap();
      auto group = grm->GetGroup();

      for(auto robot : group->GetRobots()) {
        CSpaceConstraint startConstraint(robot,startGcfg.GetRobotCfg(robot));
        startConstraints.push_back(startConstraint);
      }

      for(auto vid2 : m_modeGroundedVertices[kv1.first]) {

        // Make sure vertices are unique
        if(vid1 == vid2)
          continue;

        auto vertex2 = m_groundedHypergraph.GetVertex(vid2);

        // Create goal constraint from vertex 2
        auto goalGcfg = vertex2.property.first->GetVertex(vertex2.property.second);
        std::vector<CSpaceConstraint> goalConstraints;

        for(auto robot : group->GetRobots()) {
          CSpaceConstraint goalConstraint(robot,goalGcfg.GetRobotCfg(robot));
          goalConstraints.push_back(goalConstraint);
        }

        // Create group task
        GroupTask groupTask(group);

        for(size_t i = 0; i < goalConstraints.size(); i++) {
          // Create individual robot task

          const auto& startConstraint = startConstraints[i];
          const auto& goalConstraint = goalConstraints[i];

          auto robot = startConstraint.GetRobot();
          if(robot != goalConstraint.GetRobot())
            throw RunTimeException(WHERE) << "Mismatching robots.";

          MPTask task(robot);
          task.SetStartConstraint(std::move(startConstraint.Clone()));
          task.AddGoalConstraint(std::move(goalConstraint.Clone()));
          groupTask.AddTask(task);
        }

        // Query path for task
        lib->SetPreserveHooks(true);
        lib->Solve(prob,&groupTask,m_solution.get(),m_queryStrategy, LRand(), 
            "Query transition path");
        lib->SetPreserveHooks(false);

        // Extract cost of path from solution
        auto path = m_solution->GetGroupPath(groupTask.GetRobotGroup());
        Transition transition;
        //TODO:: Decide if this is necessary 
        transition.cost = path->Length();

        // Add arc to hypergraph
        m_groundedHypergraph.AddHyperarc({vid1},{vid2},transition);
      }
    }
  }
}

void
ModeGraph::
ApplyAction(Action* _action, std::set<std::vector<VID>>& _applied, std::vector<VID>& _newModes) {
  auto as = this->GetTMPLibrary()->GetActionSpace();

  // Extract the formation constraints
  std::vector<FormationCondition*> initialFormationConditions;
  auto initialStage = _action->GetStages()[0];
  for(auto label : _action->GetStageConditions(initialStage)) {
    auto c = as->GetCondition(label);
    auto f = dynamic_cast<FormationCondition*>(c);
    if(f)
      initialFormationConditions.push_back(f);
  }

  // Look at possible combinations of modes in the mode hyerpgraph 
  // that satisfy the formation constraint
  std::vector<std::vector<VID>> formationModes(initialFormationConditions.size());

  for(size_t i = 0; i < initialFormationConditions.size(); i++) {
    auto f = initialFormationConditions[i];

    for(const auto& kv : m_modeHypergraph.GetVertexMap()) {
      auto vid = kv.first;
      auto mode = kv.second.property;

      std::set<Robot*> used;
      for(auto type : f->GetTypes()) {
        for(auto robot : mode->robotGroup->GetRobots()) {
          // Make sure robot has not been accounted for
          if(used.count(robot))
            continue;

          // Reserve robot if it is a match
          if(robot->GetCapability() == type)
            used.insert(robot);
        }
      }

      // Check if the number of saved robots matches the required number
      if(f->GetTypes().size() == used.size() and used.size() == mode->robotGroup->Size()) 
        formationModes[i].push_back(vid);
    }
  }

  // Convert potential individual mode assignments into mode sets
  std::vector<VID> partialSet;
  auto modeSets = CollectModeSets(formationModes,0,partialSet);

  // Apply the action and generate new modes for each valid combination
  // not already in _applied

  // Extract the final formation and motion constraints
  std::vector<FormationCondition*> finalFormationConditions;
  std::vector<MotionCondition*> finalMotionConditions;
  auto finalStage = _action->GetStages().back();

  for(auto label : _action->GetStageConditions(finalStage)) {
    auto c = as->GetCondition(label);
    auto f = dynamic_cast<FormationCondition*>(c);
    if(f)
      finalFormationConditions.push_back(f);

    auto m = dynamic_cast<MotionCondition*>(c);
    if(m)
      finalMotionConditions.push_back(m);
  }

  for(auto set : modeSets) {

    // Check that this combo has not been tried before
    if(_applied.count(set))
      continue;

    // Collect all available robots
    std::vector<Robot*> robots;
    for(auto vid : set) {
      auto mode = m_modeHypergraph.GetVertexType(vid);
      for(auto robot : mode->robotGroup->GetRobots()) {
        robots.push_back(robot);
      }
    }

    // Collect robot roles
    std::unordered_map<std::string,Robot*> roleMap;
    for(size_t i = 0; i < set.size(); i++) {
      auto vid = set[i];
      auto mode = m_modeHypergraph.GetVertexType(vid);
      auto formationCondition = initialFormationConditions[i];
      State state;
      state[mode->robotGroup] = std::make_pair(nullptr,MAX_INT);
      formationCondition->AssignRoles(roleMap,state);
    }

    // Collect possible assignment of robots into groups
    std::vector<std::vector<std::vector<Robot*>>> possibleAssignments(finalFormationConditions.size());
    for(size_t i = 0; i < possibleAssignments.size(); i++) {
      auto fc = finalFormationConditions[i];
      auto types = fc->GetTypes();

      // Collect all robot matches for each type
      std::vector<std::vector<Robot*>> possibleTypeAssignments(types.size());
      for(size_t j = 0; j < types.size(); j++) {
        std::vector<Robot*> matches;

        for(auto robot : robots) {
          if(robot->GetCapability() == types[j]) {
            matches.push_back(robot);
          }
        }

        possibleTypeAssignments[j] = matches;
      }

      possibleAssignments[i] = possibleTypeAssignments;
    }

    // Build output mode sets
    std::vector<Mode*> partial;
    auto modeSetCombos = CollectModeSetCombinations(possibleAssignments,0,partial,{});

    // Add formation and path constraints
    for(auto combo : modeSetCombos) {
      for(size_t i = 0; i < combo.size(); i++) {
        auto formationCondition = finalFormationConditions[i];
        auto mode = combo[i];

        // Create formation constraints from roleMap
        auto formation = formationCondition->GenerateFormation(roleMap);
        mode->formations.insert(formation);

        // Grab path constraints from final stage
        for(auto motionCondition : finalMotionConditions) {
          auto roles = motionCondition->GetRoles();

          for(auto robot : mode->robotGroup->GetRobots()) {
            std::string role;

            for(auto kv : roleMap) {
              if(kv.second == robot) {
                role = kv.first;
                break;
              }
            }

            if(!roles.count(role))
              continue;
  
            auto constraints = motionCondition->GetConstraints(robot->GetCapability());
            for(auto c : constraints) {
              if(motionCondition->GetRole(c) != role)
                continue;

              auto constraint = c->Clone();
              mode->constraints.push_back(std::move(constraint));
            }
          }
        }
      }
    }

    // Add new modes to the graph and connect them to the start modes
    std::set<VID> tail; 
    for(auto vid : set) {
      tail.insert(vid);
    }

    for(auto modeSet : modeSetCombos) {
      std::set<VID> head;
      for(auto mode : modeSet) {
        //auto vid = m_modeHypergraph.AddVertex(mode);
        auto vid = AddMode(mode);
        head.insert(vid);
        _newModes.push_back(vid);
      }

      m_modeHypergraph.AddHyperarc(head,tail,_action);
    }

    // Mark that this combination has already been tried.
    _applied.insert(set);
  }
}

std::vector<std::vector<ModeGraph::VID>>
ModeGraph::
CollectModeSets(const std::vector<std::vector<VID>>& _formationModes, size_t _index, 
               const std::vector<VID>& _partialSet) {

  // Check if we've covered all of our formations
  if(_index == _formationModes.size())
    return {_partialSet};

  // Intialize the return vector
  std::vector<std::vector<VID>> modeSets;
  
  // Add new vids (modes) to the partial set
  auto vids = _formationModes[_index];
  for(auto vid : vids) {

    // Make sure vid is not already included in the set
    auto iter = std::find(_partialSet.begin(), _partialSet.end(), vid);
    if(iter != _partialSet.end())
      continue;

    // Add vid ot copy of partial set
    std::vector<VID> set = _partialSet;
    set.push_back(vid);

    // Recursively add additional vids
    auto sets = CollectModeSets(_formationModes,_index+1,set);

    // Add newly discovered sets to the return vector
    for(auto set : sets) {
      modeSets.push_back(set);
    }
  }

  return modeSets;
}

std::vector<std::vector<ModeGraph::Mode*>>
ModeGraph::
CollectModeSetCombinations(const std::vector<std::vector<std::vector<Robot*>>>& _possibleAssignments,
                        size_t _index, std::vector<Mode*> _partial, 
                        const std::set<Robot*>& _used) {

  // Check if we've reached the end
  if(_index == _possibleAssignments.size())
    return {_partial};
  
  // Intialize the return vector
  std::vector<std::vector<Mode*>> combos;

  // Isolate current mode
  auto possibleModeAssignments = _possibleAssignments[_index];

  // Collect possible combinations for current mode
  std::vector<Robot*> emptyPartial;
  auto modes = CollectModeCombinations(possibleModeAssignments, 0, emptyPartial, _used);

  // Recursively build rest of mode sets
  for(auto mode : modes) {
    std::vector<Mode*> partial = _partial;
    partial.push_back(mode);

    std::set<Robot*> used = _used;
    for(auto robot : mode->robotGroup->GetRobots()) {
      used.insert(robot);
    }

    auto newCombos = CollectModeSetCombinations(_possibleAssignments,_index+1,partial,used);
    for(auto combo : newCombos) {
      combos.push_back(combo);
    }
  }

  return combos;
}

std::vector<ModeGraph::Mode*>
ModeGraph::
CollectModeCombinations(const std::vector<std::vector<Robot*>>& _possibleModeAssignments,
                        size_t _index, const std::vector<Robot*> _partial,
                        const std::set<Robot*>& _used) {

  if(_index == _possibleModeAssignments.size()) {
    // Convert vector into robot group
      std::string groupLabel = "";
      for(auto robot : _partial) {
        groupLabel += (robot->GetLabel() + "--");
      }

      // Add the group to the problem and solution
      auto problem = this->GetMPProblem();
      auto group = problem->AddRobotGroup(_partial,groupLabel);
      m_solution->AddRobotGroup(group);
      
      //TODO::Figure out how to construct other features of the mode
      Mode* mode = new Mode();
      mode->robotGroup = group;
      
      return {mode};
  }

  std::vector<Mode*> combos;

  auto typeOptions = _possibleModeAssignments[_index];
  for(auto robot : typeOptions) {
    if(_used.count(robot))
      continue;

    std::set<Robot*> used = _used;
    used.insert(robot);

    auto partial = _partial;
    partial.push_back(robot);

    auto newCombos = CollectModeCombinations(_possibleModeAssignments,_index+1,
                                             partial,used);
    
    for(auto combo : newCombos) {
      combos.push_back(combo);
    }
  }

  return combos;
}

void
ModeGraph::
SaveInteractionPaths(Interaction* _interaction, State& _start, State& _end, 
                     std::unordered_map<RobotGroup*,Mode*> _startModeMap) {

  //auto problem = this->GetMPProblem();

  const auto& stages = _interaction->GetStages();
  State start;
  State end;

  Transition transition;

  // Collect set of individual paths
  for(size_t i = 1; i < stages.size(); i++) {
    auto paths = _interaction->GetToStagePaths(stages[i]);

    double stageCost = 0;

    // Collect individual robot paths
    for(auto path : paths) {
      const auto& cfgs = path->Cfgs();
      // Skip the first cfgs if this is not the first path
      for(size_t j = (i == 1) ? 0:1; j < cfgs.size(); j++) {
        transition.explicitPaths[path->GetRobot()].push_back(cfgs[j]);
      }

      // Update max cost at this stage
      stageCost = std::max(stageCost,path->Length());
    }

    transition.cost += stageCost;
  }

  // Add the start state to the grounded vertices graph
  auto tail = AddStateToGroundedHypergraph(start,_startModeMap);

  // TODO::Construct end modes
  std::unordered_map<RobotGroup*,Mode*> endModeMap;

  // Add the end state to the grounded vertices graph
  auto head = AddStateToGroundedHypergraph(_end,endModeMap);

  // Save transition in hypergraph
  m_groundedHypergraph.AddHyperarc(head,tail,transition);


/*
  for(size_t i = 1; i < stages.size(); i++) {
    auto paths = _interaction->GetToStagePaths(stages[i]);

    if(i == 1) {
      // Initialize start from original modeSet
      start = _start;
    }
    else {
      // Collect robots in group
      std::vector<Robot*> robots;
      std::string groupLabel = "";
      for(auto path : paths) {
        auto robot = path->GetRobot();
        robots.push_back(robot);
        groupLabel += (robot->GetLabel() + "--");
      }

      // Add the group to the problem and solution
      RobotGroup* group = problem->AddRobotGroup(robots,groupLabel);
      m_solution->AddRobotGroup(group);
      start[group] = std::make_pair(nullptr,MAX_INT);
    }

    // Collect start and goal cfgs
    std::unordered_map<Robot*,Cfg> startCfgs;
    for(auto path : paths) {
      auto robot = path->GetRobot();

      const auto& cfgs = path->Cfgs();
      
      if(cfgs.empty())
        throw RunTimeException(WHERE) << "Expected path for "
                                        << robot->GetLabel()
                                        << " in iteraction "
                                        << _interaction->GetLabel()
                                        << ", stage:"
                                        << stages[i];

      // Save the start cfg
      auto startCfg = cfgs.front();
      startCfgs[robot] = startCfg;
    }

    // Update start state to hold discovered vertices
    for(auto& kv : start) {
      auto group = kv.first;
      auto grm = m_solution->GetGroupRoadmap(group);
      GroupCfg gcfg(grm);

      for(auto robot : group->GetRobots()) {
        auto cfg = startCfgs[robot];
        gcfg.SetRobotCfg(robot,std::move(cfg));
      }

      // Add new group cfg to the group roadmap
      auto vid = grm->AddVertex(gcfg);

      // Set the state for the group
      kv.second = std::make_pair(grm,vid);
    }

    // Add the start state to the grounded vertices graph
    auto head = AddStateToGroundedHypergraph(start,_startModeMap);

    // Check if this is the first stage or not
    if(!tail.empty()) {
      // Connect it to the previous stage
      m_groundedHypergraph.AddHyperarc(head,tail,transition);
    }

    // Save the transition to the next stage
    transition = Transition();
    for(auto path : paths) {
      transition.explicitPaths[path->GetRobot()] = path;
      transition.cost = std::max(transition.cost,path->Length());
    }

    // Set this as the tail for the next stage
    tail = head;
  }

  // Add final end state to grounded hypergraph
  auto head = AddStateToGroundedHypergraph(_end,_startModeMap);
  m_groundedHypergraph.AddHyperarc(head,tail,transition);
  */
}

ModeGraph::VID
ModeGraph::
AddMode(Mode* _mode) {

  // Check if mode already exists
  for(const auto& mode : m_modes) {
    // Check if robot groups are the same
    if(mode->robotGroup != _mode->robotGroup)
      continue;

    // Check if both have same number of formations and constraints
    auto formations1 = mode->formations;
    auto formations2 = _mode->formations;

    if(formations1.size() != formations2.size())
      continue;

    const auto& constraints1 = mode->constraints;
    const auto& constraints2 = _mode->constraints;

    if(constraints1.size() != constraints2.size())
      continue;

    // Check if the formations are the same
    bool fMatch = true;
    for(auto f1 : formations1) {

      bool match = false;

      for(auto f2 : formations2) {
        if(*f1 == *f2) {
          match = true;
          break;
        }
      }

      if(!match) {
        fMatch = false;
        break;
      }
    }

    if(!fMatch)
      continue;

    // Check if constraints are the same
    bool cMatch = true;
    for(const auto& c1 : constraints1) {
      
      bool match = false;

      auto b1 = dynamic_cast<BoundaryConstraint*>(c1.get());
  
      for(const auto& c2 : constraints2) {
        auto b2 = dynamic_cast<BoundaryConstraint*>(c2.get());

        if(*b1 == *b2) {
          match = true;
          break;
        }
      }

      if(!match) {
        cMatch = false;
        break;
      }
    }

    if(!cMatch)
      continue;
    
    // Found existing copy of mode already saved
    return m_modeHypergraph.GetVID(mode.get());
  }

  auto mode = std::unique_ptr<Mode>(_mode);
  m_modes.push_back(std::move(mode));

  return m_modeHypergraph.AddVertex(m_modes.back().get());
}

std::set<ModeGraph::VID>
ModeGraph::
AddStateToGroundedHypergraph(const State& _state, std::unordered_map<RobotGroup*,Mode*> _modeMap) {

  std::set<VID> vids;

  for(const auto& kv : _state) {
    auto mode = _modeMap[kv.first];
    auto mvid = m_modeHypergraph.GetVID(mode);

    auto gvid = m_groundedHypergraph.AddVertex(kv.second);
    m_modeGroundedVertices[mvid].insert(gvid);

    vids.insert(gvid);
  }

  return vids;
}
/*----------------------------------------------------------------------------*/
