#include "ModeGraph.h"

#include "Behaviors/Agents/Coordinator.h"

#include "ConfigurationSpace/GroupLocalPlan.h"

#include "MPProblem/Constraints/BoundaryConstraint.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"

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

  m_unactuatedSM = _node.Read("unactuatedSM",true,"",
               "Sampler Method to use to generate unactuated cfgs.");

  m_querySM = _node.Read("querySM",true,"",
               "Sampler Method to use to generate query cfgs.");

  m_queryStrategy = _node.Read("queryStrategy",true,"",
                      "MPStrategy label to query roadaps.");

  m_queryStrategyStatic = _node.Read("queryStrategyStatic",false,m_queryStrategy,
                      "MPStrategy label to query roadaps without sampling and connecting new start/goals.");

  m_expansionStrategy = _node.Read("expansionStrategy",true,"",
                      "MPStrategy label to query roadaps.");
  m_numUnactuatedSamples = _node.Read("numUnactuatedSamples",false,0,0,1000,
                      "The number of samples to generate for each unactuated mode.");
  m_numInteractionSamples = _node.Read("numInteractionSamples",false,1,1,1000,
                      "The number of samples to generate for each transtion.");
  m_maxAttempts = _node.Read("maxAttempts",false,1,1,1000,
                      "The max number of attempts to generate a sample.");

  m_writeHypergraphs = _node.Read("writeHypergraphs",false,m_writeHypergraphs,
                      "Flag to write hypergraphs to output files.");
}

ModeGraph::
~ModeGraph() {}

/*-------------------------------- Interface ---------------------------------*/

void
ModeGraph::
Initialize() {

  auto problem = this->GetMPProblem();

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Initialize");

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
ModeGraph::
GenerateRepresentation(const State& _start) {

  auto initialModes = AddStartState(_start);

  // Mode hypergraph
  GenerateModeHypergraph(initialModes);
  
  // Grounded hypergraph
  GroundedVertex origin = std::make_pair(nullptr,0);
  auto originVID = m_groundedHypergraph.AddVertex(origin);
  GroundedVertex goal = std::make_pair(nullptr,MAX_INT);
  auto goalVID = m_groundedHypergraph.AddVertex(goal);

  std::set<VID> startVIDs;
  std::set<VID> goalVIDs;

  if(m_debug) {
    std::cout << "MODE HYPERGRAPH" << std::endl;
    m_modeHypergraph.Print();
  }

  do {

    SampleNonActuatedCfgs(_start,startVIDs,goalVIDs);
    SampleTransitions();

    GenerateRoadmaps(_start,startVIDs,goalVIDs);
    ConnectTransitions();

    Transition fromOrigin;
    fromOrigin.cost = -1;
    if(m_groundedHypergraph.GetHID(startVIDs,{originVID}) == MAX_INT)
      m_groundedHypergraph.AddHyperarc(startVIDs,{originVID},fromOrigin);

    //Transition toGoal;
    //m_groundedHypergraph.AddHyperarc({goalVID},goalVIDs,toGoal);
    ConfigureGoalSets(goalVID, goalVIDs);

  } while(!ContainsSolution(startVIDs));

  if(m_debug) {
    std::cout << "MODE HYPERGRAPH" << std::endl;
    m_modeHypergraph.Print();
    std::cout << "GROUNDED HYPERGRAPH" << std::endl;
    m_groundedHypergraph.Print();
    std::cout << "GROUNDED HYPERARC COSTS" << std::endl;
    for(auto kv : m_groundedHypergraph.GetHyperarcMap()) {
      auto hid = kv.first;
      auto hyperarc = kv.second;
      std::cout << hid << " : " << hyperarc.property.cost << std::endl;
    }
  }

  if(m_writeHypergraphs) {
    std::string base = this->GetMPProblem()->GetBaseFilename();

    m_modeHypergraph.Print(base + "-mode-hypergraph.hyp");
    m_groundedHypergraph.Print(base + "-grounded-hypergraph.hyp");
  }
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

MPSolution* 
ModeGraph::
GetMPSolution() {
  return m_solution.get();
}
    
ModeGraph::VID 
ModeGraph::
GetModeOfGroundedVID(const VID& _vid) const {
  for(const auto& kv : m_modeGroundedVertices) {
    if(kv.second.count(_vid))
      return kv.first;
  }
  return MAX_INT;
}

/*---------------------------- Helper Functions ------------------------------*/

std::vector<ModeGraph::VID>
ModeGraph::
AddStartState(const State& _start) {

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

  return newModes;
}

void
ModeGraph::
GenerateModeHypergraph(const std::vector<VID>& _initialModes) {
  auto as = this->GetTMPLibrary()->GetActionSpace();

  auto newModes = _initialModes;

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
SampleNonActuatedCfgs(const State& _start, std::set<VID>& _startVIDs, std::set<VID>& _goalVIDs) {

  auto plan = this->GetPlan();
  auto decomp = plan->GetDecomposition();
  auto lib = this->GetMPLibrary();
  auto uaSM = lib->GetSampler(m_unactuatedSM);
  auto qSM = lib->GetSampler(m_querySM);
  lib->SetMPSolution(m_solution.get());

  // Set all robots and objects to virtual
  auto c = this->GetPlan()->GetCoordinator();
  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {
      r->SetVirtual(true);
    }
  }
 
  for(auto& kv : m_modeHypergraph.GetVertexMap()) {
    // Check if mode is unactuated
    auto mode = kv.second.property;

    bool unactuated = true;
    for(auto robot : mode->robotGroup->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive())
        continue;
      unactuated = false;
      break;
    }

    if(!unactuated)
      continue;

    for(auto robot : mode->robotGroup->GetRobots()) {
      robot->SetVirtual(false);
    }

    m_unactuatedModes.insert(kv.first);

    // Ensure robot group is in the mp solution
    m_solution->AddRobotGroup(mode->robotGroup);
    auto grm = m_solution->GetGroupRoadmap(mode->robotGroup);

    // Configure MPLibrary
    GroupTask gt(mode->robotGroup);
    lib->SetTask(nullptr);
    lib->SetGroupTask(&gt);

    // Check if mode is in the start state
    auto iter = _start.find(mode->robotGroup);

    if(iter != _start.end()) {

      // Add start cfg to grounded hypergraph
      auto state = _start.at(mode->robotGroup);
      auto gcfg = state.first->GetVertex(state.second);
      auto vid = grm->AddVertex(gcfg);
      GroundedVertex gv = std::make_pair(grm,vid);

      auto groundedVID = m_groundedHypergraph.AddVertex(gv);
      m_modeGroundedVertices[kv.first].insert(groundedVID);
      _startVIDs.insert(groundedVID);
      m_entryVertices.insert(groundedVID);
    }

    // Check if mode is in the goal conditions
    for(auto st : decomp->GetGroupMotionTasks()) {
      auto task = st->GetGroupMotionTask().get();
      if(!task or task->GetRobotGroup() != mode->robotGroup)
        continue;

      // Sample goal cfg
      std::map<Robot*, const Boundary*> boundaryMap;
      for(auto iter = task->begin(); iter != task->end(); iter++) {
        auto c = dynamic_cast<BoundaryConstraint*>(iter->GetGoalConstraints()[0].get());
        auto b = c->GetBoundary();
        boundaryMap[iter->GetRobot()] = b;
      }

      std::vector<GroupCfg> samples;
      qSM->Sample(1,m_maxAttempts,boundaryMap,std::back_inserter(samples));
      
      if(samples.size() == 0)
        throw RunTimeException(WHERE) << "Unable to generate goal configuration for "
                                      << mode->robotGroup->GetLabel()
                                      << ".";
      // Add goal cfg to grounded hypergraph
      auto gcfg = samples[0].SetGroupRoadmap(grm);
      auto vid = grm->AddVertex(gcfg);
      auto gv = std::make_pair(grm,vid);

      auto groundedVID = m_groundedHypergraph.AddVertex(gv);
      m_modeGroundedVertices[kv.first].insert(groundedVID);
      _goalVIDs.insert(groundedVID);
      m_exitVertices.insert(groundedVID);
      m_goalVertexTaskMap[st] = groundedVID;
    }

    // Sample other cfgs and add to grounded hypergraph
    auto b = this->GetMPProblem()->GetEnvironment()->GetBoundary(); 
    std::vector<GroupCfg> samples;
    uaSM->Sample(m_numUnactuatedSamples,m_maxAttempts,b,std::back_inserter(samples));
    
    for(auto sample : samples) { 
      // Add sample to grounded hypergraph
      auto gcfg = sample.SetGroupRoadmap(grm);
      auto vid = grm->AddVertex(gcfg);
      auto gv = std::make_pair(grm,vid);

      auto groundedVID = m_groundedHypergraph.AddVertex(gv);
      m_modeGroundedVertices[kv.first].insert(groundedVID);
      m_entryVertices.insert(groundedVID);
      m_exitVertices.insert(groundedVID);
    }
  }
  
  // Set all robots and objects to virtual
  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {
      r->SetVirtual(false);
    }
  }
}

void
ModeGraph::
ConfigureGoalSets(const size_t& _sink, std::set<VID>& _goalVIDs) {
 
  // TODO::Find sets of valid goals to solve decomposition
  std::vector<std::set<VID>> goalSets;

  // TODO::Compile sets of completed tasks that satisfy the decomposition
  // Currently assume one depth of OR values 
  std::set<SemanticTask*> seen;
  std::vector<std::set<SemanticTask*>> buckets;
  for(auto kv : m_goalVertexTaskMap) {
    auto st = kv.first;

    if(seen.count(st))
      continue;

    seen.insert(st);
    std::set<SemanticTask*> bucket = {st};

    auto parent = st->GetParent();
    auto type = parent->GetSubtaskRelation();
    
    if(type == SemanticTask::SubtaskRelation::XOR) {
      for(auto sibling : parent->GetSubtasks()) {
        bucket.insert(sibling);
        seen.insert(sibling);
      }
    }

    buckets.push_back(bucket);
  }
 
  auto taskSets = BuildTaskSets({},0,buckets);

  for(auto set : taskSets) {
    std::set<size_t> goalSet;
    for(auto st : set) {
      auto vid = m_goalVertexTaskMap[st];
      goalSet.insert(vid);
    }
    goalSets.push_back(goalSet);
  }

  // Add transition from each set to the sink
  for(auto set : goalSets) {
    Transition toGoal;
    if(m_groundedHypergraph.GetHID({_sink},set) == MAX_INT)
      m_groundedHypergraph.AddHyperarc({_sink},set,toGoal);
  } 
}

std::vector<std::set<SemanticTask*>>
ModeGraph::
BuildTaskSets(std::set<SemanticTask*> _taskSet, size_t _index, 
              const std::vector<std::set<SemanticTask*>>& _buckets) {
  
  if(_index == _buckets.size()) {
    if(m_debug) {
      std::cout << "Found Task Set:" << std::endl;
      for(auto st : _taskSet) {
        std::cout << "\t" << st->GetLabel();
      }
      std::cout << std::endl;
    }
    return {_taskSet};
  }

  auto bucket = _buckets[_index];

  std::vector<std::set<SemanticTask*>> newSets;

  for(auto st : bucket) {
    auto taskSet = _taskSet;
    taskSet.insert(st);
    auto sets = BuildTaskSets(taskSet,_index+1,_buckets);
    for(auto set : sets) {
      newSets.push_back(set);
    }
  }

  return newSets;
}

void
ModeGraph::
SampleTransitions() {

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SampleTransitions");

  // Set all robots and objects to virtual
  auto c = this->GetPlan()->GetCoordinator();
  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {
      r->SetVirtual(true);
    }
  }

  // For each edge in the mode graph, generate n samples
  for(auto& kv : m_modeHypergraph.GetHyperarcMap()) {

    auto& hyperarc = kv.second;
    m_groundedInstanceTracker[kv.first] = 0;
    
    // Check if hyperarc is a reversed action, and only plan
    // the forward actions as the reverse will also be saved
    if(hyperarc.property.second)
      continue;

    auto interaction = dynamic_cast<Interaction*>(hyperarc.property.first);

    State modeSet;
    std::unordered_map<RobotGroup*,Mode*> tailModeMap;
    std::unordered_map<RobotGroup*,Mode*> headModeMap;
    std::set<std::pair<size_t,RobotGroup*>> unactuatedModes;

    for(auto vid : hyperarc.tail) {
      auto mode = m_modeHypergraph.GetVertex(vid).property;
      modeSet[mode->robotGroup] = std::make_pair(nullptr,MAX_INT);
      tailModeMap[mode->robotGroup] = mode;

      if(m_unactuatedModes.count(vid)) {
        unactuatedModes.insert(std::make_pair(vid,mode->robotGroup));
      }
    }

    for(auto vid : hyperarc.head) {
      auto mode = m_modeHypergraph.GetVertex(vid).property;
      headModeMap[mode->robotGroup] = mode;
    }

    auto label = interaction->GetInteractionStrategyLabel();
    auto is = this->GetInteractionStrategyMethod(label);

    // Set robots involved as non virtual
    for(auto kv : modeSet) {
      auto group = kv.first;
      for(auto r : group->GetRobots()) {
        r->SetVirtual(false);
      }
    }

    bool foundInteraction = false;
    bool canReach = false;
    // If this hyperarc involves an unactuated mode, use the grounded vertices
    if(!unactuatedModes.empty()) {
      if(unactuatedModes.size() > 1)
        throw RunTimeException(WHERE) << "Multiple unactuated modes in an "
                      "interaction not currently supported.";

      auto unactuatedVID = unactuatedModes.begin()->first;
      auto unactuatedGroup = unactuatedModes.begin()->second;

      auto groundedVertices = m_modeGroundedVertices[unactuatedVID];

      //for(auto gv : m_modeGroundedVertices[unactuatedVID]) {
      for(auto iter = groundedVertices.begin(); iter != groundedVertices.end(); iter++) {
        // Get grounded vertex
        auto groundedVertex = m_groundedHypergraph.GetVertexType(*iter);

        auto startSet = modeSet;
        startSet[unactuatedGroup] = groundedVertex;

        // Check if robot can even reach pose
        if(!CanReach(startSet))
          continue;

        canReach = true;

        for(size_t j = 0; j < m_maxAttempts; j++) {
          // Make state copy and add grounded vertex to pass to IS.
          // Will get overwritten as goal state
          auto goalSet = startSet;
          if(!is->operator()(interaction,goalSet))
            continue;

          foundInteraction = true;

          m_groundedInstanceTracker[kv.first] = m_groundedInstanceTracker[kv.first] + 1;
          // Save interaction paths
          SaveInteractionPaths(interaction,modeSet,goalSet,tailModeMap,headModeMap);
          break;
        }
      }
    }
    // Otherwise, sample completely new grounded vertices
    else {

      for(size_t i = 0; i < m_numInteractionSamples; i++) {

        for(size_t j = 0; j < m_maxAttempts; j++) {

          // Make state copy to pass by ref and get output state
          auto goalSet = modeSet;

          // Check if robots can even reach to perform interaction
          if(!CanReach(goalSet))
            continue;

          canReach = true;

          if(!is->operator()(interaction,goalSet))
            continue;

          foundInteraction = true;

          m_groundedInstanceTracker[kv.first] = m_groundedInstanceTracker[kv.first] + 1;

          // Save interaction paths
          SaveInteractionPaths(interaction,modeSet,goalSet,tailModeMap,headModeMap);
          break;
        }
      }
    }

    // Set robots involved as back to virtual
    for(auto kv : modeSet) {
      auto group = kv.first;
      for(auto r : group->GetRobots()) {
        r->SetVirtual(true);
      }
    }

    if(m_debug and !foundInteraction and canReach) {
      std::cout << "Failed to find interaction for " << interaction->GetLabel()
                << " with starting mode";
      for(auto kv : modeSet) {
        auto group = kv.first;
        std::cout << " " << group->GetLabel();
      }
      std::cout << std::endl;
      std::cout << std::endl;
    }
  }

  // Set all robots and objects to back to non-virtual
  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {
      r->SetVirtual(false);
    }
  }

  if(m_debug) {
    std::cout << "Grounding instance count of each mode hyperarc." << std::endl;
    for(auto kv : m_groundedInstanceTracker) {
      auto hid = kv.first;
      auto count = kv.second;
      std::cout << hid << " : " << count << std::endl;
    }
  }
}

void
ModeGraph::
GenerateRoadmaps(const State& _start, std::set<VID>& _startVIDs, std::set<VID>& _goalVIDs) {

  auto plan = this->GetPlan();
  auto c = plan->GetCoordinator();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GenerateRoadmaps");

  auto decomp = plan->GetDecomposition();
  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();
  auto qSM = lib->GetSampler(m_querySM);
  lib->SetMPSolution(m_solution.get());

  // Set all robots to virtual
  for(auto pair : c->GetInitialRobotGroups()) {
    auto group = pair.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(true);
    }
  }

  for(auto& kv : m_modeHypergraph.GetVertexMap()) {
    // Check if mode is actuated
    auto mode = kv.second.property;
    if(m_unactuatedModes.count(kv.first))
      continue;

    auto grm = m_solution->GetGroupRoadmap(mode->robotGroup);

    // Check if mode is in the start state
    auto iter = _start.find(mode->robotGroup);

    // If mode is initial mode, add starting vertex
    if(iter != _start.end()) {

      // Add start cfg to grounded hypergraph
      auto state = _start.at(mode->robotGroup);
      auto gcfg = state.first->GetVertex(state.second).SetGroupRoadmap(grm);
      auto vid = grm->AddVertex(gcfg);
      GroundedVertex gv = std::make_pair(grm,vid);

      auto groundedVID = m_groundedHypergraph.AddVertex(gv);
      m_modeGroundedVertices[kv.first].insert(groundedVID);
      _startVIDs.insert(groundedVID);
      m_entryVertices.insert(groundedVID);
    }

    // Check if mode is in the goal conditions
    for(auto st : decomp->GetGroupMotionTasks()) {
      auto task = st->GetGroupMotionTask().get();
      if(!task or task->GetRobotGroup() != mode->robotGroup)
        continue;

      // Sample goal cfg
      std::map<Robot*, const Boundary*> boundaryMap;
      for(auto iter = task->begin(); iter != task->end(); iter++) {
        auto c = dynamic_cast<BoundaryConstraint*>(iter->GetGoalConstraints()[0].get());
        auto b = c->GetBoundary();
        boundaryMap[iter->GetRobot()] = b;
      }

      std::vector<GroupCfg> samples;
      qSM->Sample(1,m_maxAttempts,boundaryMap,std::back_inserter(samples));
      
      if(samples.size() == 0)
        throw RunTimeException(WHERE) << "Unable to generate goal configuration for"
                                      << mode->robotGroup->GetLabel()
                                      << ".";
      // Add goal cfg to grounded hypergraph
      auto gcfg = samples[0].SetGroupRoadmap(grm);
      auto vid = grm->AddVertex(gcfg);
      auto gv = std::make_pair(grm,vid);

      auto groundedVID = m_groundedHypergraph.AddVertex(gv);
      m_modeGroundedVertices[kv.first].insert(groundedVID);
      _goalVIDs.insert(groundedVID);
      m_exitVertices.insert(groundedVID);
    }
  }

  /*for(const auto& kv : _start) {
    auto group = kv.first;
    auto grm = kv.second.first;
    auto vid = kv.second.second;
    auto gcfg = grm->GetVertex(vid);

    auto newGrm = m_solution->GetGroupRoadmap(group);
    auto newGcfg = gcfg.SetGroupRoadmap(newGrm);
    newGrm->AddVertex(newGcfg);
  }*/

  /*
  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();
  */
 
  // For each actuated mode in the mode hypergraph, run the expansion strategy
  for(auto kv : m_modeHypergraph.GetVertexMap()) {
    auto vertex = kv.second;
    auto mode = vertex.property;

    bool actuated = false;
    for(auto robot : mode->robotGroup->GetRobots()) {
      if(!robot->GetMultiBody()->IsPassive()) {
        actuated = true;
        break;
      }
    }

    if(!actuated)
      continue;

    // Initialize dummy task
    auto task = new GroupTask(mode->robotGroup);

    for(auto r : mode->robotGroup->GetRobots()) {
      auto t = MPTask(r);

      // Add start constraints
      auto startCfg = prob->GetInitialCfg(r);
      auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,startCfg));
      t.SetStartConstraint(std::move(startConstraint));

      // Add mode/path constraints
      for(const auto& c : mode->constraints) {
        if(c->GetRobot() != r)
          continue;
        t.AddPathConstraint(std::move(c->Clone()));
      }
      task->AddTask(t);
    } 

    // Set active formation constraints
    auto formations = mode->formations;
    auto grm = m_solution->GetGroupRoadmap(mode->robotGroup);
    grm->SetAllFormationsInactive();
    for(auto f : formations) {
      grm->SetFormationActive(f);
    }

    // Set robots not virtual
    for(auto robot : grm->GetGroup()->GetRobots()) {
      robot->SetVirtual(false);
    }

    // Call the MPLibrary solve function to expand the roadmap
    lib->SetPreserveHooks(true);
    lib->Solve(prob,task,m_solution.get(),m_expansionStrategy, LRand(), 
            "ExpandModeRoadmap");
    lib->SetPreserveHooks(false);

    // Set robots not virtual
    for(auto robot : grm->GetGroup()->GetRobots()) {
      robot->SetVirtual(true);
    }

    delete task;
  }

  // TODO::Collect set of modes for each robot-unique object type pairing

  // TODO::For each pairing of individual robot to object type, build an initial roadmap

  // TODO::Copy roadmap to all instances of individual robot-unique object type pairs

  // Set all robots back to non-virtual
  for(auto pair : c->GetInitialRobotGroups()) {
    auto group = pair.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(false);
    }
  }
}

void
ModeGraph::
ConnectTransitions() {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer* mt = new MethodTimer(stats,this->GetNameAndLabel() + "::ConnectTransitions");

  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();

  // Set robots virtual
  for(const auto& robot : prob->GetRobots()) {
    robot->SetVirtual(true);
  }

  // For each actuated mode in the mode hypergraph, attempt to connect grounded transition samples
  for(auto kv1 : m_modeHypergraph.GetVertexMap()) {
  
    if(m_unactuatedModes.count(kv1.first))
      continue;

    auto mode = m_modeHypergraph.GetVertexType(kv1.first);

    for(auto vid1 : m_modeGroundedVertices[kv1.first]) {

      if(!m_entryVertices.count(vid1))
        continue;

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

        if(!m_exitVertices.count(vid2))
          continue;

        if(m_debug) {
          std::cout << "Attempting to connect grounded vertices: "
                    << vid1 << " -> " << vid2 << std::endl;
        }

        auto vertex2 = m_groundedHypergraph.GetVertex(vid2);

        // Create goal constraint from vertex 2
        auto goalGcfg = vertex2.property.first->GetVertex(vertex2.property.second);
        std::vector<CSpaceConstraint> goalConstraints;

        for(auto robot : group->GetRobots()) {
          CSpaceConstraint goalConstraint(robot,goalGcfg.GetRobotCfg(robot));
          goalConstraints.push_back(goalConstraint);
        }

        // Create group task
        auto groupTask = std::shared_ptr<GroupTask>(new GroupTask(group));

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

          for(const auto& c : mode->constraints) {
            if(c->GetRobot() != robot)
              continue;
            task.AddPathConstraint(std::move(c->Clone()));
          }

          groupTask->AddTask(task);
        }

        // Set active formation constraints
        auto formations = mode->formations;
        auto grm = m_solution->GetGroupRoadmap(group);
        grm->SetAllFormationsInactive();
        for(auto f : formations) {
          grm->SetFormationActive(f);
        }

        // Set robots not virtual
        for(auto robot : grm->GetGroup()->GetRobots()) {
          robot->SetVirtual(false);
        }

        if(m_debug) {
          auto cfg1 = vertex1.property.first->GetVertex(vertex1.property.second);
          auto cfg2 = vertex2.property.first->GetVertex(vertex2.property.second);
          std::cout << "Querying path for " << cfg1.GetGroupRoadmap()->GetGroup()->GetLabel() << std::endl;
          std::cout << "\tFrom: " << cfg1.PrettyPrint() << std::endl;
          std::cout << "\tTo: " << cfg2.PrettyPrint() << std::endl;
        }

        // Query path for task
        lib->SetPreserveHooks(true);
        lib->Solve(prob,groupTask.get(),m_solution.get(),m_queryStrategy, LRand(), 
            "Query transition path");
        lib->SetPreserveHooks(false);

        grm->SetAllFormationsInactive();

        // Set robots back to virtual
        for(auto robot : grm->GetGroup()->GetRobots()) {
          robot->SetVirtual(true);
        }

        // Extract cost of path from solution
        auto path = m_solution->GetGroupPath(groupTask->GetRobotGroup());

        if(m_debug and !path->Empty()) {
          std::cout << "Path for transition: " << vid1 << " -> " << vid2 << std::endl;
          for(const auto& cfg : path->Cfgs()) {
            std::cout << "\t" << cfg.PrettyPrint() << std::endl;
          }
          std::cout << std::endl;
        }

        if(path->Empty()) {
          if(m_debug) {
            std::cout << "Failed to find a path for: " 
                      << vid1 
                      << " -> " 
                      << vid2 
                      << std::endl;
          }
          continue;
        }

        Transition transition;
        transition.taskSet.push_back({groupTask});
        transition.cost = path->TimeSteps();
        //transition.cost = 100;
        transition.taskFormations[groupTask.get()] = formations;

        // Add arc to hypergraph
        m_groundedHypergraph.AddHyperarc({vid2},{vid1},transition,true);
      }
    }
  }

  // Set robots virtual
  for(const auto& robot : prob->GetRobots()) {
    if(this->GetPlan()->GetCoordinator()->GetRobot() == robot.get())
      continue;
    robot->SetVirtual(false);
  }

  delete mt;
  // Strange behavior to avoid issues with roadmaps being improved after costs are saved
  if(m_queryStrategy != m_queryStrategyStatic) {
    auto stash = m_queryStrategy;
    m_queryStrategy = m_queryStrategyStatic;
    ConnectTransitions();
    m_queryStrategy = stash;
  }

}

void
ModeGraph::
ApplyAction(Action* _action, std::set<std::vector<VID>>& _applied, std::vector<VID>& _newModes) {
  auto as = this->GetTMPLibrary()->GetActionSpace();

  // Extract the formation and motion constraints
  std::vector<FormationCondition*> initialFormationConditions;
  std::vector<MotionCondition*> initialMotionConditions;
  auto initialStage = _action->GetStages()[0];
  for(auto label : _action->GetStageConditions(initialStage)) {
    auto c = as->GetCondition(label);
    auto f = dynamic_cast<FormationCondition*>(c);
    if(f) {
      initialFormationConditions.push_back(f);
      continue;
    }
    auto m = dynamic_cast<MotionCondition*>(c);
    if(m)
      initialMotionConditions.push_back(m);
  }

  // Connect motion constraints
  std::unordered_map<std::string,Constraint*> motionConstraintMap;
  for(auto m : initialMotionConditions) {
    for(auto role : m->GetRoles()) {
      auto constraint = m->GetRoleConstraint(role);
      motionConstraintMap[role] = constraint;
    }
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
      if(f->GetTypes().size() != used.size() or used.size() != mode->robotGroup->Size()) 
        continue;

      // Check if mode meets formation requirements
      // Create state
      State state;
      state[mode->robotGroup] = std::make_pair(nullptr,MAX_INT);
      std::unordered_map<std::string,Robot*> roleMap;
      f->AssignRoles(roleMap,state);
      bool satisfied = mode->formations.empty();
      for(auto formation : mode->formations) {
        if(f->DoesFormationMatch(roleMap,formation)) {
          satisfied = true;
          break;
        }
      }

      if(!satisfied)
        continue;

      // Check if mode meets motion constraints
      for(auto role : f->GetRoles()) {
        // Check if role has associated motion constraint
        auto iter1 = motionConstraintMap.find(role);
        if(iter1 == motionConstraintMap.end())
          continue;

        // Check if constraint is in the mode
        auto constraint = motionConstraintMap[role];
        auto b1 = constraint->GetBoundary();
        if(!b1)
          continue;

        for(const auto& c : mode->constraints) {
          // Check if boundaries are the same
          auto b2 = c->GetBoundary();
          if(b1->Type() == b2->Type()
            and b1->Name() == b2->Name()
            and b1->GetDimension() == b2->GetDimension()
            and b1->GetCenter() == b2->GetCenter()) {

            bool match = true;
            for(size_t i = 0; i < b1->GetDimension(); i++) {
              if(!(b1->GetRange(i) == b2->GetRange(i))) {
                match = false;
              }
            }
            if(match)
              continue;
          }

          satisfied = false;
          break;
        }
      }

      if(satisfied) {
        formationModes[i].push_back(vid);
      }
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
    /*std::unordered_map<std::string,Robot*> roleMap;
    for(size_t i = 0; i < set.size(); i++) {
      auto vid = set[i];
      auto mode = m_modeHypergraph.GetVertexType(vid);
      auto formationCondition = initialFormationConditions[i];
      State state;
      state[mode->robotGroup] = std::make_pair(nullptr,MAX_INT);
      formationCondition->AssignRoles(roleMap,state);
    }*/

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

        // Construct role map for mode set
        std::unordered_map<std::string,Robot*> roleMap;
        State state;
        state[mode->robotGroup] = std::make_pair(nullptr,MAX_INT);
        formationCondition->AssignRoles(roleMap,state);

        // Create formation constraints from roleMap
        auto formation = formationCondition->GenerateFormation(roleMap);
        if(formation)
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
              constraint->SetRobot(robot);
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

      // TODO::Make sure this doesn't happen in the first place
      // Make sure there is not any overlap in head and tail
      bool overlap = false;

      std::set<VID> head;
      for(auto mode : modeSet) {
        //auto vid = m_modeHypergraph.AddVertex(mode);
        auto oldSize = m_modeHypergraph.Size();
        auto vid = AddMode(mode);

        if(tail.count(vid)) {
          overlap = true;
          break;
        }
        head.insert(vid);
        if(oldSize < m_modeHypergraph.Size())
          _newModes.push_back(vid);
      }

      if(overlap)
        continue;

      m_modeHypergraph.AddHyperarc(head,tail,std::make_pair(_action,false));
      if(_action->IsReversible()) {
        m_modeHypergraph.AddHyperarc(tail,head,std::make_pair(_action,true));
      }
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

    // Make sure this new mode does not intersect with the partial set
    auto mode = m_modeHypergraph.GetVertexType(vid);
    const auto& robots = mode->robotGroup->GetRobots();
    bool intersect = false;
    for(auto vid2 : _partialSet) {
      auto mode2 = m_modeHypergraph.GetVertexType(vid2);
      for(auto r1 : robots) {
        for(auto r2 : mode2->robotGroup->GetRobots()) {
          if(r1 == r2) {
            intersect = true;
            break;
          }
        }
        if(intersect)
          break;
      }
      if(intersect)
        break;
    }
    if(intersect)
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
                     std::unordered_map<RobotGroup*,Mode*> _startModeMap,
                     std::unordered_map<RobotGroup*,Mode*> _endModeMap) {

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SaveInteractionPaths");
  //auto problem = this->GetMPProblem();

  const auto& stages = _interaction->GetStages();
  State start = _start;
  State end = _end;


  Transition transition;

  // Collect set of individual paths
  for(size_t i = 1; i < stages.size(); i++) {
    auto paths = _interaction->GetToStagePaths(stages[i]);

    auto solution = _interaction->GetToStageSolution(stages[i]);

    double stageCost = 0;

    // Collect individual robot paths
    /*std::unordered_map<Robot*,size_t> startVIDs;
    std::unordered_map<Robot*,size_t> endVIDs;
    std::unordered_map<Robot*,double> individualWeights;
    */

    for(auto path : paths) {
      const auto& cfgs = path->Cfgs();
  
      auto robot = path->GetRobot();
      /*auto rm = m_solution->GetRoadmap(robot);

      auto startVID = grm->AddVertex(cfgs.front());
      auto goalVID = grm->AddVertex(cfgs.back());

      startVIDs[robot] = startVID;      
      endVIDs[robot] = goalVID;
      individualWeights[robot] = path->TimeSteps();
      */

      // Skip the first cfgs if this is not the first path
      bool isFirst = transition.explicitPaths[robot].size() == 0;
      for(size_t j = (isFirst) ? 0:1; j < cfgs.size(); j++) {
        transition.explicitPaths[robot].push_back(cfgs[j]);
      }

      // Update max cost at this stage
      stageCost = std::max(stageCost,double(path->TimeSteps()));
    }

    transition.cost += stageCost;

    // Collect stage tasks
    auto tasks = _interaction->GetToStageTasks(stages[i]);
    transition.taskSet.push_back(tasks);
    // Grab active formations from interaction solutions
    for(auto task : tasks) {
      auto grm = solution->GetGroupRoadmap(task->GetRobotGroup());
      auto formations = grm->GetActiveFormations();

      if(grm->GetGroup()->Size() > 1 and formations.empty())
        throw RunTimeException(WHERE) << "AT THE MOMENT, THIS SHOULD NEVER HAPPEN.";

      transition.taskFormations[task.get()] = formations;
    }
    
    /*// Add an edge in the underlying group roadmaps for these paths
    for(auto task : tasks) {
      auto group = task->GetRobotGroup();
      auto grm = m_solution->GetGroupRoadmap(group);
      if(!grm) {
        m_solution->AddRobotGroup(group); 
        grm = m_solution->GetGroupRoadmap(group);
      }

      GroupCfg start(grm);
      GroupCfg end(grm);
      double weight = 0;
      for(auto robot : group->GetRobots()) {
        start.SetRobotCfg(robot,std::move(startCfgs[robot]));
        end.SetRobotCfg(robot,std::move(endCfgs[robot]));
        weight = std::max(weight,individualWeights[robot]);
      }

      auto startVID = grm->AddVertex(start);
      auto goalVID = grm->AddVertex(end);

      GroupLocalPlan<Cfg> edge(grm);
      edge.SetWeight(weight);
      grm->AddEdge(startVID,goalVID,edge);
    }*/

    // Copy the mp solution info into the local solution
    auto toStageSolution = _interaction->GetToStageSolution(stages[i]);
    for(auto task : tasks) {
      auto group = task->GetRobotGroup();

      std::vector<std::unordered_map<size_t,size_t>> vertexMaps;

      // Copy individual robot roadmaps
      for(auto robot : group->GetRobots()) {
        auto localRM = m_solution->GetRoadmap(robot);
        auto interRM = toStageSolution->GetRoadmap(robot);

        vertexMaps.push_back({});
        std::unordered_map<size_t,size_t>& vertexMap = vertexMaps.back();

        // Copy vertices
        for(auto vit = interRM->begin(); vit != interRM->end(); vit++) {
          auto oldVID = vit->descriptor();
          auto cfg = vit->property();
          auto newVID = localRM->AddVertex(cfg);
          vertexMap[oldVID] = newVID;
        }

        // Copy edges
        for(auto vit = interRM->begin(); vit != interRM->end(); vit++) {
          for(auto eit = vit->begin(); eit != vit->end(); eit++) {
            auto source = vertexMap[eit->source()];
            auto target = vertexMap[eit->target()];
            auto edge = eit->property();
            localRM->AddEdge(source,target,edge);
          }
        }
      }

      // Copy group roadmaps
      auto localGrm = m_solution->GetGroupRoadmap(group);
      auto interGrm = toStageSolution->GetGroupRoadmap(group);
    
      std::unordered_map<size_t,size_t> groupVertexMap;

      // Copy vertices
      for(auto vit = interGrm->begin(); vit != interGrm->end(); vit++) {
        auto oldVID = vit->descriptor();
        auto oldGcfg = vit->property();

        localGrm->SetAllFormationsInactive();
        for(auto f : oldGcfg.GetFormations()) {
          localGrm->AddFormation(f);
        }

        // Construct group cfg
        GroupCfg newGcfg(localGrm);
        for(size_t i = 0; i < group->GetRobots().size(); i++) {
          auto oldVID = oldGcfg.GetVID(i);

          if(oldVID != MAX_INT) {
            newGcfg.SetRobotCfg(i,vertexMaps[i][oldVID]);
          }
          else {
            auto cfg = oldGcfg.GetRobotCfg(i);
            newGcfg.SetRobotCfg(i,std::move(cfg));
          }
        }

        // Copy it over to local group roadmap
        auto newVID = localGrm->AddVertex(newGcfg);
        groupVertexMap[oldVID] = newVID;
      }

      // Copy edges
      for(auto vit = interGrm->begin(); vit != interGrm->end(); vit++) {
        for(auto eit = vit->begin(); eit != vit->end(); eit++) {
          auto source = groupVertexMap[eit->source()];
          auto target = groupVertexMap[eit->target()];
          auto oldEdge = eit->property();

          // Reconstruct edge in local group roadmap
          GroupLocalPlan<Cfg> newEdge(localGrm);

          newEdge.SetLPLabel(oldEdge.GetLPLabel());

          auto& edgeDescriptors = oldEdge.GetEdgeDescriptors();
          for(size_t i = 0; i < group->GetRobots().size(); i++) {
            auto oldEd = edgeDescriptors[i];

            if(oldEd.source() != MAX_INT and oldEd.target() != MAX_INT) {
              auto source = vertexMaps[i][oldEd.source()];
              auto target = vertexMaps[i][oldEd.target()];
              GroupLocalPlanType::ED ed(source,target);
              newEdge.SetEdge(group->GetRobots()[i],ed);
            }
            else {
              auto edge = (*oldEdge.GetEdge(i));
              newEdge.SetEdge(i,std::move(edge));
            }
          }
        
          newEdge.SetWeight(oldEdge.GetWeight());
          newEdge.SetTimeSteps(oldEdge.GetTimeSteps());

          localGrm->AddEdge(source,target,newEdge);
        }
      }
    }
  }

  std::unordered_map<Robot*,Cfg> startCfgs;
  std::unordered_map<Robot*,Cfg> endCfgs;

  // Grab start and end cfgs
  for(auto kv : transition.explicitPaths) {
    auto robot = kv.first;
    auto cfg = kv.second[0];
    startCfgs[robot] = cfg;
    cfg = kv.second.back();
    endCfgs[robot] = cfg;
  }

  // Update start state
  for(auto kv : start) {
    auto group = kv.first;
    auto grm = m_solution->GetGroupRoadmap(group);
    // Set mode formations
    grm->SetAllFormationsInactive();
    for(auto f : _startModeMap[group]->formations) {
      grm->SetFormationActive(f);
    }

    GroupCfg gcfg(grm);
    for(auto robot : group->GetRobots()) {
      gcfg.SetRobotCfg(robot,std::move(startCfgs[robot]));
    }

    auto vid = grm->AddVertex(gcfg);
    start[group] = std::make_pair(grm,vid);

    grm->SetAllFormationsInactive();
  }

  // Update end state 
  for(auto kv : end) {
    auto group = kv.first;
    auto grm = m_solution->GetGroupRoadmap(group);
    // Set mode formations
    grm->SetAllFormationsInactive();
    for(auto f : _endModeMap[group]->formations) {
      grm->SetFormationActive(f);
    }

    GroupCfg gcfg(grm);
    for(auto robot : group->GetRobots()) {
      gcfg.SetRobotCfg(robot,std::move(endCfgs[robot]));
    }

    auto vid = grm->AddVertex(gcfg);
    end[group] = std::make_pair(grm,vid);

    grm->SetAllFormationsInactive();
  }

  // Add the start state to the grounded vertices graph
  auto tail = AddStateToGroundedHypergraph(start,_startModeMap);
  for(auto v : tail) {
    m_exitVertices.insert(v);
  }

  // Add the end state to the grounded vertices graph
  auto head = AddStateToGroundedHypergraph(end,_endModeMap);
  for(auto v : head) {
    m_entryVertices.insert(v);
  }

  // Make sure transition does not already exist in graph
  if(m_groundedHypergraph.GetHID(head,tail) == MAX_INT) {
    // Save transition in hypergraph
    m_groundedHypergraph.AddHyperarc(head,tail,transition);
  }

  if(_interaction->IsReversible()) {

    for(auto v : tail) {
      m_entryVertices.insert(v);
    }
    for(auto v : head) {
      m_exitVertices.insert(v);
    }

    Transition reverse;

    // Reverse explicit paths
    for(auto kv : transition.explicitPaths) {
      auto path = kv.second;
      std::reverse(path.begin(), path.end());
      reverse.explicitPaths[kv.first] = path;
    }

    // Reverse implicit paths
    for(auto kv : transition.implicitPaths) {
      auto path = std::make_pair(kv.second.second.second,kv.second.second.first);
      reverse.implicitPaths[kv.first] = std::make_pair(kv.second.first,path);
    }

    // Reverse tasks
    for(auto stage : transition.taskSet) {
      std::vector<std::shared_ptr<GroupTask>> newTasks;
      for(auto task : stage) {
        auto newTask = std::shared_ptr<GroupTask>(new GroupTask(task->GetRobotGroup()));
        for(auto iter = task->begin(); iter != task->end(); iter++) {
          auto start = iter->GetGoalConstraints()[0]->Clone();
          auto goal  = iter->GetStartConstraint()->Clone();
          MPTask t(iter->GetRobot());
          t.SetStartConstraint(std::move(start));
          t.AddGoalConstraint(std::move(goal));
          newTask->AddTask(t);
        }
        newTasks.push_back(newTask);
      }
      reverse.taskSet.push_back(newTasks);
    }
    std::reverse(reverse.taskSet.begin(),reverse.taskSet.end());
    reverse.cost = transition.cost;

    // Save reverse transition in hypergraph
    if(m_groundedHypergraph.GetHID(tail,head) == MAX_INT) {
      // Save transition in hypergraph
      m_groundedHypergraph.AddHyperarc(tail,head,reverse);
    }
  }


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
      transition.cost = std::max(transition.cost,path->TimeSteps());
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
        if(!f1 or !f2) {
          if(f1 == f2) {
            match = true;
            break;
          }
        }
        else if(*f1 == *f2) {
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

    // Check if grounded vertex violates mode
    auto gcfg = kv.second.first->GetVertex(kv.second.second);
    for(auto& c : mode->constraints) {
      auto robot = c->GetRobot();
      auto cfg = gcfg.GetRobotCfg(robot);
      if(!c->Satisfied(cfg)) {
        throw RunTimeException(WHERE) << robot->GetLabel() 
                                      << " violated mode constraints at : " 
                                      << cfg
                                      << std::endl;
      }
    }

    vids.insert(gvid);
  }

  return vids;
}

bool
ModeGraph::
CanReach(const State& _state) {
  
  // Construct bounding spheres
  std::vector<std::pair<Vector3d,double>> spheres;

  for(const auto& kv : _state) {
    auto group = kv.first;
    auto rm = kv.second.first;
    auto vid = kv.second.second;
    
    if(rm and vid != MAX_INT) {
      auto gcfg = rm->GetVertex(vid);
      gcfg.ConfigureRobot();
    }

    for(auto robot : group->GetRobots()) {
      auto mb = robot->GetMultiBody();

      Vector3d center = mb->GetBase()->GetWorldTransformation().translation();
      double radius = rm ? 0 : std::numeric_limits<double>::infinity();

      if(!mb->IsPassive()) {
        // TODO::Compute proper radius - for now, we know UR5e is roughly 1 meter
        radius = .91;
      }

      spheres.emplace_back(center,radius);
    }
  }

  // Check if all spheres intersect
  for(size_t i = 0; i < spheres.size()-1; i++) {
    const auto& sphere1 = spheres[i];
    const auto& center1 = sphere1.first;
    const auto& radius1 = sphere1.second;
    
    for(size_t j = i+1; j < spheres.size(); j++) {
      const auto& sphere2 = spheres[j];
      const auto& center2 = sphere2.first;
      const auto& radius2 = sphere2.second;

      double distance = 0;
      for(size_t k = 0; k < 3; k++) {
        distance += std::pow((center1[k]-center2[k]),2);
      }
      distance = sqrt(distance);

      if(distance > radius1 + radius2)
        return false;
    }
  }
  
  return true;
}

bool
ModeGraph::
ContainsSolution(std::set<VID>& _startVIDs) {

  // Run a dijkstra search backwards through hypergraph as if it was a graph

  // Get graph representation grounded hypergraph
  auto g = m_groundedHypergraph.GetReverseGraph();

  // Setup dijkstra functions
  SSSPTerminationCriterion<ModeGraph::GroundedHypergraph::GraphType> termination(
    [this](typename ModeGraph::GroundedHypergraph::GraphType::vertex_iterator& _vi,
           const SSSPOutput<typename ModeGraph::GroundedHypergraph::GraphType>& _sssp) {
    const auto& vertex = _vi->property();
    auto grm = vertex.first;
    if(!grm)
      return SSSPTermination::Continue;

    auto group = grm->GetGroup();

    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive())
        return SSSPTermination::Continue;
    }

    return SSSPTermination::EndBranch;
  });

  SSSPPathWeightFunction<ModeGraph::GroundedHypergraph::GraphType> weight(
    [this,g](typename ModeGraph::GroundedHypergraph::GraphType::adj_edge_iterator& _ei,
           const double _sourceDistance,
           const double _targetDistance) {

    auto target = _ei->target();
    auto grm = g->GetVertex(target).first;

    bool hasObject = false;

    if(grm) {
      auto group = grm->GetGroup();
      for(auto robot : group->GetRobots()) {
        if(robot->GetMultiBody()->IsPassive()) {
          hasObject = true;
          break;
        }
      }
    }

    if(!hasObject and grm)
      return std::numeric_limits<double>::infinity();

    auto groundedHA = _ei->property();
    double edgeWeight = groundedHA.cost;

    //TODO::Decide if this is what we want
    //edgeWeight = std::min(1.,edgeWeight);

    double newDistance = _sourceDistance + edgeWeight;
    return newDistance;
  });

  // Run dijkstra backwards from sink
  std::vector<size_t> starts = {1};
  auto output = DijkstraSSSP(g,starts,weight,termination);

  // Ensure each object start vertex can reach the sink
  for(auto v : _startVIDs) {

    auto vertex = m_groundedHypergraph.GetVertexType(v);
    auto group = vertex.first->GetGroup();
    bool passive = false;

    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive()) {
        passive = true;
        break;
      }
    }

    if(!passive)
      continue;

    auto iter = output.distance.find(v);
    if(iter == output.distance.end())
      return false;
  }

  return true;
}

/*----------------------------------------------------------------------------*/

istream&
operator>>(istream& _is, const ModeGraph::Mode* _mode) {
  return _is;
}

ostream&
operator<<(ostream& _os, const ModeGraph::Mode* _mode) {
  return _os;
}

istream&
operator>>(istream& _is, const ModeGraph::ReversibleAction _ra) {
  return _is;
}

ostream&
operator<<(ostream& _os, const ModeGraph::ReversibleAction _ra) {
  return _os;
}

istream&
operator>>(istream& _is, const ModeGraph::GroundedVertex _vertex) {
  return _is;
}

ostream&
operator<<(ostream& _os, const ModeGraph::GroundedVertex _vertex) {
  return _os;
}

istream&
operator>>(istream& _is, const ModeGraph::Transition _t) {
  return _is;
}

ostream&
operator<<(ostream& _os, const ModeGraph::Transition _t) {
  return _os;
}
