#include "ModeGraph.h"

#include "MPProblem/MPProblem.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/InteractionStrategies/InteractionStrategyMethod.h"

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
    auto mode = kv.first;
    auto vid = m_modeHypergraph.AddVertex(mode);
    newModes.push_back(vid);
  }

  // Keep track of already expanded mode/action combinations
  std::unordered_map<Action*,std::set<std::set<VID>>> appliedActions;

  do {
    
    // Add new modes to the hyerpgraph
    for(auto vid : newModes) {
      m_modeGroundedVertices[vid] = std::unordered_set<VID>();
    }

    // Apply actions to discovered modes to make new modes
    for(auto actionLabel : as->GetActions()) {
      auto action = actionLabel.second;
      newModes = ApplyAction(action,appliedActions[action]);
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
    for(auto vid : hyperarc.tail) {
      Mode* mode = m_modeHypergraph.GetVertex(vid).property;
      modeSet[mode] = std::make_pair(nullptr,MAX_INT);
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
        SaveInteractionPaths(interaction,modeSet,goalSet);
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
    auto mode = kv.first;
    auto grm = kv.second.first;
    auto vid = kv.second.second;
    auto gcfg = grm->GetVertex(vid);

    auto newGrm = m_solution->GetGroupRoadmap(mode);
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
    auto task = new GroupTask(mode);

    for(auto r : mode->GetRobots()) {
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

  // For each mode in the mode hypergraph, attempt to connect transition samples
  for(auto kv1 : m_groundedHypergraph.GetVertexMap()) {
    auto vid1 = kv1.first;
    auto vertex1 = kv1.second;
    auto mode = vertex1.property.first;

    // Create start constraint from vertex
    auto startGcfg = vertex1.property.first->GetVertex(vertex1.property.second);
    std::vector<CSpaceConstraint> startConstraints;

    auto grm = startGcfg.GetGroupRoadmap();
    auto group = grm->GetGroup();

    for(auto robot : group->GetRobots()) {
        CSpaceConstraint startConstraint(robot,startGcfg.GetRobotCfg(robot));
        startConstraints.push_back(startConstraint);
    }

    for(auto kv2 : m_groundedHypergraph.GetVertexMap()) {
      auto vid2 = kv2.first;

      // Make sure vertices are unique
      if(vid1 == vid2)
          continue;

      // Make sure vertices of the same mode
      auto vertex2 = kv2.second;
      if(mode != vertex2.property.first)
          continue;

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

std::vector<ModeGraph::VID>
ModeGraph::
ApplyAction(Action* _action, std::set<std::set<VID>>& _applied) {
  auto as = this->GetTMPLibrary()->GetActionSpace();

  std::vector<VID> newModes;

  // Extract the formation constraints
  std::vector<FormationCondition*> formationConditions;
  auto initialStage = _action->GetStages()[0];
  for(auto label : _action->GetStageConditions(initialStage)) {
    auto c = as->GetCondition(label);
    auto f = dynamic_cast<FormationCondition*>(c);
    if(f)
      formationConditions.push_back(f);
  }

  // Look at possible combinations of modes in the mode hyerpgraph 
  // that satisfy the formation constraint
  std::vector<std::vector<VID>> formationModes(formationConditions.size());

  for(size_t i = 0; i < formationConditions.size(); i++) {
    auto f = formationConditions[i];

    for(const auto& kv : m_modeHypergraph.GetVertexMap()) {
      auto vid = kv.first;
      auto mode = kv.second.property;

      std::set<Robot*> used;
      for(auto type : f->GetTypes()) {
        for(auto robot : mode->GetRobots()) {
          // Make sure robot has not been accounted for
          if(used.count(robot))
            continue;

          // Reserve robot if it is a match
          if(robot->GetCapability() == type)
            used.insert(robot);
        }
      }

      // Check if the number of saved robots matches the required number
      if(f->GetTypes().size() == used.size()) 
        formationModes[i].push_back(vid);
    }
  }

  // Convert potential individual mode assignments into mode sets
  std::set<VID> partialSet;
  auto modeSets = CollectModeSets(formationModes,0,partialSet);

  // Apply the action and generate new modes for each valid combination
  // not already in _applied

  // Extract the final formation constraints
  formationConditions.clear();
  auto finalStage = _action->GetStages().back();
  for(auto label : _action->GetStageConditions(finalStage)) {
    auto c = as->GetCondition(label);
    auto f = dynamic_cast<FormationCondition*>(c);
    if(f)
      formationConditions.push_back(f);
  }

  for(auto set : modeSets) {

    // Check that this combo has not been tried before
    if(_applied.count(set))
      continue;

    // Collect all available robots
    std::vector<Robot*> robots;
    for(auto vid : set) {
      auto mode = m_modeHypergraph.GetVertexType(vid);
      for(auto robot : mode->GetRobots()) {
        robots.push_back(robot);
      }
    }

    // Collect possible assignment of robots into groups
    std::vector<std::vector<std::vector<Robot*>>> possibleAssignments(formationConditions.size());
    for(size_t i = 0; i < possibleAssignments.size(); i++) {
      auto fc = formationConditions[i];
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

    // Add new modes to the graph and connect them to the start modes
    const std::set<VID>& tail = set; 
    for(auto modeSet : modeSetCombos) {
      std::set<VID> head;
      for(auto mode : modeSet) {
        auto vid = m_modeHypergraph.AddVertex(mode);
        head.insert(vid);
        newModes.push_back(vid);
      }

      m_modeHypergraph.AddHyperarc(tail,head,_action);
    }

    // Mark that this combination has already been tried.
    _applied.insert(tail);
  }

  return newModes;
}

std::vector<std::set<ModeGraph::VID>>
ModeGraph::
CollectModeSets(const std::vector<std::vector<VID>>& _formationModes, size_t _index, 
               const std::set<VID>& _partialSet) {

  // Check if we've covered all of our formations
  if(_index == _formationModes.size())
    return {_partialSet};

  // Intialize the return vector
  std::vector<std::set<VID>> modeSets;
  
  // Add new vids (modes) to the partial set
  auto vids = _formationModes[_index];
  for(auto vid : vids) {

    // Make sure vid is not already included in the set
    if(_partialSet.count(vid))
      continue;

    // Add vid ot copy of partial set
    auto set = _partialSet;
    set.insert(vid);

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
    auto partial = _partial;
    partial.push_back(mode);

    auto used = _used;
    for(auto robot : mode->GetRobots()) {
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
      Mode* mode = problem->AddRobotGroup(_partial,groupLabel);
      m_solution->AddRobotGroup(mode);
  }

  std::vector<Mode*> combos;

  auto typeOptions = _possibleModeAssignments[_index];
  for(auto robot : typeOptions) {
    if(_used.count(robot))
      continue;

    auto used = _used;
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
SaveInteractionPaths(Interaction* _interaction, State& _start, State& _end) {
  auto problem = this->GetMPProblem();

  const auto& stages = _interaction->GetStages();
  State start;
  State end;

  std::set<VID> tail;
  Transition transition;

  for(size_t i = 1; i < stages.size(); i++) {
    auto paths = _interaction->GetToStagePaths(stages[i]);

    if(i == 0) {
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
    auto head = AddStateToGroundedHypergraph(start);

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
  auto head = AddStateToGroundedHypergraph(_end);
  m_groundedHypergraph.AddHyperarc(head,tail,transition);
}

std::set<ModeGraph::VID>
ModeGraph::
AddStateToGroundedHypergraph(const State& _state) {

  std::set<VID> vids;

  for(const auto& kv : _state) {
    auto mode = kv.first;
    auto mvid = m_modeHypergraph.GetVID(mode);

    auto gvid = m_groundedHypergraph.AddVertex(kv.second);
    m_modeGroundedVertices[mvid].insert(gvid);

    vids.insert(gvid);
  }

  return vids;
}
/*----------------------------------------------------------------------------*/
