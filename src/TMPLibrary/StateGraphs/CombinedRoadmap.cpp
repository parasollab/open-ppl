#include "CombinedRoadmap.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/ChildAgent.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/Solution/Plan.h"

#include "Utilities/MetricUtils.h"


/*------------------------------ Construction --------------------------------*/
CombinedRoadmap::
CombinedRoadmap(){
	this->SetName("CombinedRoadmap");
};

CombinedRoadmap::
CombinedRoadmap(XMLNode& _node) : StateGraph(_node) {
	this->SetName("CombinedRoadmap");
}

CombinedRoadmap::
~CombinedRoadmap() {
  for(auto sm : m_semanticRoadmaps) {
    delete sm;
  } 
}
/*------------------------------ Construction -------------------------------*/

void
CombinedRoadmap::
Initialize(){
  auto prob = this->GetMPProblem();

  // Intialize hypergraph with source and sink
  m_hypergraph = std::unique_ptr<TMPHypergraph>(new TMPHypergraph());
  TMPVertex source, sink;
  auto sourceVID = m_hypergraph->AddVertex(source);
  auto sinkVID = m_hypergraph->AddVertex(sink);

  std::set<size_t> sourceHeadSet;

  if(m_debug) {
    std::cout << "Source VID: " << sourceVID << std::endl;
    std::cout << "Sink VID: " << sinkVID << std::endl;
  }

  // Initialize mpsolution from coordinator
  auto c = this->GetPlan()->GetCoordinator();
  m_mpSolution = std::unique_ptr<MPSolution>(new MPSolution(c->GetRobot()));

  std::unordered_map<Robot*,size_t> robotStartVIDMap;

  for(auto& kv : c->GetInitialRobotGroups()) {

    auto group = kv.first;
    auto formation = kv.second;

    // Add individial robots to mp solution
    for(auto& r : group->GetRobots()) {
      m_mpSolution->AddRobot(r);
    }

    // Create new group roadmaps in mp solution
    //auto grm = new GroupRoadmapType(group.get(), m_mpSolution.get());
    m_mpSolution->AddRobotGroup(group);
    auto grm = m_mpSolution->GetGroupRoadmap(group);

    // Add the initial formation to the roadmap and set it active.
    if(formation) {
      grm->AddFormation(formation);
      grm->SetFormationActive(formation);
    }

    // Create new semantic roadmap from group roadmap
    auto sr = AddSemanticRoadmap(grm,ActionUpdate());

    // Create initial group vertex
    auto gcfg = GroupCfg(grm);

    // Add initial cfg to individual roadmaps
    for(auto& r : group->GetRobots()) {
      auto rm = m_mpSolution->GetRoadmap(r);
      auto cfg = prob->GetInitialCfg(r);
      auto vid = rm->AddVertex(cfg);
      robotStartVIDMap[r] = vid;
     
      // Update group vertex 
      gcfg.SetRobotCfg(r,vid);
    }

    // Add intial group vertex and connect to virtual source
    auto rvid = grm->AddVertex(gcfg);

    auto hvid = m_vertexMap[sr][rvid];
    if(m_debug) {
      std::cout << "Hypergraph vid: " << hvid << std::endl;
    }

    sourceHeadSet.insert(hvid);

    // Add group and position to initial state
    m_initialState[group] = std::make_pair(grm,rvid);
  }

  TMPHyperarc arc;
  arc.semantic = true;
  
  // Setup 'paths' for initial cfgs
  for(auto kv : robotStartVIDMap) {
    auto robot = kv.first;
    auto rm = m_mpSolution->GetRoadmap(robot);
    auto path = new Path(rm);
    *path += {kv.second};
    arc.paths.push_back(path);
  }

  m_hypergraph->AddHyperarc(sourceHeadSet,{sourceVID},arc);
}

/*-------------------------------- Accessors --------------------------------*/

void    
CombinedRoadmap::
AddInteraction(CompositeSemanticRoadmap _csr, State _input, State _output, Interaction* _inter) {

  // Save formation constraints before things get moved
  std::unordered_map<RobotGroup*,std::unordered_set<Formation*>> outputFormations;
  for(auto kv : _output) {
    outputFormations[kv.first] = kv.second.first->GetActiveFormations();
  }

  // Make sure input and output states reflect proper roadmap.
  RemapState(_input);
  // Set expansion status to true so that hooks get executed on the output state.
  m_expansionStatus = true;
  RemapState(_output);
  // Set expansion status back to false so none of the interaction path info gets added by accident.
  m_expansionStatus = false;

  // Create semantic roadmap for interaction.

  /*
  // Grab group roadmap.
  auto group = _inter->GetToInterimPath()->GetRoadmap()->GetGroup();
  m_mpSolution->AddRobotGroup(group);
  auto grm = m_mpSolution->GetGroupRoadmap(group);
  */

  // Collect updates from accross semantic roadmaps
  ActionUpdate update; 
  for(auto sr : _csr) {
    update = MergeActionUpdates(update, sr->second);
  }

  // Add new change to action update
  update.updates.push_back(std::make_pair(_input,_output));

  // Get hvids from start state
  std::set<size_t> currentHVids;
  for(auto sr : _csr) {
    auto group = sr->first->GetGroup();
    auto vid = _input[group].second;
    
    auto hvid = m_vertexMap[sr][vid];
    currentHVids.insert(hvid);
  }

  const auto& stages = _inter->GetStages();
  auto currentStage = stages[0];

  // Connect all of the intermediate stages
  for(size_t i = 1; i < stages.size()-1; i++) {
    TMPHyperarc arc;
    arc.semantic = true;
    m_interactionSolutions.push_back(_inter->ExtractToStageSolution(currentStage));

    // Pull individual robot paths from solution and add them to the hyperarc.
    for(auto& kv : _input) {
      auto group = kv.first;
      auto robots = group->GetRobots();
      for(auto robot : robots) {
        auto path = m_interactionSolutions.back()->GetPath(robot);
        path = MovePathToMPSolution(path);
        arc.paths.push_back(path);
      }
    }

    // Create virtual interaction node for next stage
    TMPVertex interactionVertex(MAX_INT,nullptr);
    size_t interactionHVid = m_hypergraph->AddVertex(interactionVertex); 

    // Connect current and next stage
    m_hypergraph->AddHyperarc({interactionHVid},currentHVids,arc);

    // Move current stage values forward
    currentStage = stages[i];
    currentHVids = {interactionHVid};
  }

  // Connect final stage
  const auto& finalStage = stages.back();

  TMPHyperarc arc;
  arc.semantic = true; 

  // Extract to Post solution form interaction.
  m_interactionSolutions.push_back(_inter->ExtractToStageSolution(finalStage));

  // Pull individual robot paths from solution and add them to the hyperarc.
  for(auto& kv : _output) {
    auto group = kv.first;
    auto robots = group->GetRobots();
    for(auto robot : robots) {
      auto path = m_interactionSolutions.back()->GetPath(robot);
      path = MovePathToMPSolution(path);
      arc.paths.push_back(path);
    }
  }

  // Create semantic roadmaps for output
  std::set<size_t> postHVids;
  for(auto kv : _output) {
    auto outputGroup = kv.second.first->GetGroup();
    m_mpSolution->AddRobotGroup(outputGroup);
    auto outputRm = m_mpSolution->GetGroupRoadmap(outputGroup);

    for(auto formation : outputFormations[outputGroup]) {
      outputRm->AddFormation(formation);
    }

    auto newSr = AddSemanticRoadmap(outputRm, update);

    auto subVID = kv.second.second;
    auto hvid = m_vertexMap[newSr][subVID];

    postHVids.insert(hvid);
  }

  // Add the connecting hyperarc.
  m_hypergraph->AddHyperarc(postHVids,currentHVids,arc);

  // Check if the hypergraph has reached a satisfying state.
  CheckForGoalState(postHVids);


  
  /*
  // Initialize incoming interaction hyperarc
  TMPHyperarc incoming;
  incoming.semantic = true; 

  // Extract to Interim solution from interaction.
  m_interactionSolutions.push_back(_inter->ExtractToInterimSolution());

  // Pull individual robot paths from solution and add them to the hyperarc.
  for(auto& kv : _input) {
    auto group = kv.first;
    auto robots = group->GetRobots();
    for(auto robot : robots) {
      auto path = m_interactionSolutions.back()->GetPath(robot);
      path = MovePathToMPSolution(path);
      incoming.paths.push_back(path);
    }
  }
 
  // Create virtual interaction node
  TMPVertex interactionVertex(MAX_INT,nullptr);
  size_t interactionHVid = m_hypergraph->AddVertex(interactionVertex); 

  // Connect start vertices and interaction vertex
  m_hypergraph->AddHyperarc({interactionHVid},startHVids,incoming);

  // Initialize outgoing interaction hyperarc
  TMPHyperarc outgoing;
  outgoing.semantic = true; 

  // Extract to Post solution form interaction.
  m_interactionSolutions.push_back(_inter->ExtractToPostSolution());

  // Pull individual robot paths from solution and add them to the hyperarc.
  for(auto& kv : _output) {
    auto group = kv.first;
    auto robots = group->GetRobots();
    for(auto robot : robots) {
      auto path = m_interactionSolutions.back()->GetPath(robot);
      path = MovePathToMPSolution(path);
      outgoing.paths.push_back(path);
    }
  }

  // Create semantic roadmaps for output
  std::set<size_t> postHVids;
  for(auto kv : _output) {
    auto outputGroup = kv.second.first->GetGroup();
    m_mpSolution->AddRobotGroup(outputGroup);
    auto outputRm = m_mpSolution->GetGroupRoadmap(outputGroup);

    for(auto formation : outputFormations[outputGroup]) {
      outputRm->AddFormation(formation);
    }

    auto newSr = AddSemanticRoadmap(outputRm, update);

    auto subVID = kv.second.second;
    auto hvid = m_vertexMap[newSr][subVID];

    postHVids.insert(hvid);
  }

  // Connect interaction vertex and end vertices
  m_hypergraph->AddHyperarc(postHVids,{interactionHVid},outgoing);
  CheckForGoalState(postHVids);
  */
  /*
  SemanticRoadmap* sr = new SemanticRoadmap(std::make_pair(grm,update));
  AddInteractionRoadmap(sr);

  // Copy interim path over
  auto interimVIDs = _inter->GetToInterimPath()->VIDs();
  auto interimRm = _inter->GetToInterimSolution()->GetGroupRoadmap(group);

  // Add first cfg to path.
  auto cfg = interimRm->GetVertex(interimVIDs[0]);
  auto previousVID = MoveGroupCfg(cfg,grm);
  auto interimPathStartVID = m_vertexMap[sr][previousVID];

  // Iterate through path and add vertices and edges.
  for(size_t i = 1; i < interimVIDs.size(); i++) {
    cfg = interimRm->GetVertex(interimVIDs[i]);
    auto vid = MoveGroupCfg(cfg,grm);

    MoveGroupEdge(interimVIDs[i-1], interimVIDs[i],
                  previousVID,vid,
                  interimRm,grm);

    previousVID = vid;
  }

  auto interimPathEndVID = m_vertexMap[sr][previousVID];

  // Copy post path over
  auto postVIDs = _inter->GetToPostPath()->VIDs();
  auto postRm = _inter->GetToPostSolution()->GetGroupRoadmap(group);

  // Add first cfg to path.
  cfg = postRm->GetVertex(postVIDs[0]);
  previousVID = MoveGroupCfg(cfg,grm);

  auto postPathStartVID = m_vertexMap[sr][previousVID];

  // Iterate through path and add vertices and edges.
  for(size_t i = 1; i < postVIDs.size(); i++) {
    cfg = postRm->GetVertex(postVIDs[i]);
    auto vid = MoveGroupCfg(cfg,grm);

    MoveGroupEdge(postVIDs[i-1], postVIDs[i],
                  previousVID,vid,
                  postRm,grm);
    
    previousVID = vid;
  }

  auto postPathEndVID = m_vertexMap[sr][previousVID];

  // Connect interim to post path
  TMPHyperarc arc;
  arc.semantic = true;
  m_hypergraph->AddHyperarc({postPathStartVID},{interimPathEndVID},arc);

  // Connect interaction path to existing semantic roadmaps
  std::set<size_t> tail;
  for(auto tailSr : _csr) {
    auto grm = tailSr->first;
    auto group = grm->GetGroup();

    auto rvid = _input[group].second;
    auto hvid = m_vertexMap[tailSr][rvid];

    tail.insert(hvid);
  }

  std::set<size_t> head = {interimPathStartVID};

  m_hypergraph->AddHyperarc(head,tail,arc);

  // Create semantic roadmaps for output
  head = {};
  for(auto kv : _output) {
    auto outputGroup = kv.second.first->GetGroup();
    m_mpSolution->AddRobotGroup(outputGroup);
    auto outputRm = m_mpSolution->GetGroupRoadmap(outputGroup);

    auto newSr = AddSemanticRoadmap(outputRm, update);

    auto subVID = kv.second.second;
    auto hvid = m_vertexMap[newSr][subVID];

    head.insert(hvid);
  }
  
  tail = {postPathEndVID};

  m_hypergraph->AddHyperarc(head,tail,arc);
  CheckForGoalState(tail);
  */
}

    
Hypergraph<CombinedRoadmap::TMPVertex,CombinedRoadmap::TMPHyperarc>* 
CombinedRoadmap::
GetHypergraph() {
  return m_hypergraph.get();
}

    
const std::set<CombinedRoadmap::SemanticRoadmap*>& 
CombinedRoadmap::
GetSemanticRoadmaps() const {
  return m_semanticRoadmaps;
}

MPSolution* 
CombinedRoadmap::
GetMPSolution() const {
  return m_mpSolution.get();
}
    
void
CombinedRoadmap::
AddRobotGroup(RobotGroup* _group) {
  m_mpSolution->AddRobotGroup(_group);
}
    
void
CombinedRoadmap::
SetExpansionStatus(bool _status) {
  m_expansionStatus = _status;
}
    
bool
CombinedRoadmap::
GetExpansionStatus() {
  return m_expansionStatus;
}

/*------------------------------ Helper Functions ----------------------------*/

CombinedRoadmap::SemanticRoadmap*
CombinedRoadmap::
AddSemanticRoadmap(GroupRoadmapType* _grm, const ActionUpdate& _update) {
  // Check if semantic roadmap already exists
  for(auto sr : m_semanticRoadmaps) {
    if(sr->first != _grm)
      continue;

    if(sr->second == _update)
      return sr;
 
    // Check for equivalent action updates
    auto state1 = ApplyActionUpdate(m_initialState,sr->second);
    auto state2 = ApplyActionUpdate(m_initialState,_update);

    if(state1 == state2)
      return sr;
  }

  auto sr = new SemanticRoadmap(std::make_pair(_grm,_update));
  m_semanticRoadmaps.insert(sr);

  if(!m_cspaceRoadmaps.count(sr->first)) {
    std::cout << "New cspace roadmap" << std::endl;
  }
  m_cspaceRoadmaps.insert(sr->first);

  sr->first->InstallHook(GroupRoadmapType::HookType::AddVertex, 
      "CombinedRoadmap::AddVertex"+std::to_string(m_hookCounter),
      [this, sr](VI _vi) {
        if(this->GetExpansionStatus()) {
          this->AddHypergraphVertex(sr,_vi);
        }
  });
  sr->first->InstallHook(GroupRoadmapType::HookType::AddEdge, 
      "CombinedRoadmap::AddEdge"+std::to_string(m_hookCounter),
      [this, sr](EI _ei) {
        if(this->GetExpansionStatus()) {
          this->AddHypergraphArc(sr,_ei);
        }
  });
  sr->first->InstallHook(GroupRoadmapType::HookType::DeleteVertex, 
      "CombinedRoadmap::DeleteVertex"+std::to_string(m_hookCounter),
      [this, sr](VI _vi) {

  });
  sr->first->InstallHook(GroupRoadmapType::HookType::DeleteEdge, 
      "CombinedRoadmap::DeleteEdge"+std::to_string(m_hookCounter),
      [this, sr](EI _ei) {

  });

  m_hookCounter++;

  // Adding the current content of the semantic roadmap to the Hypergraph.
  for(auto iter = sr->first->begin(); iter != sr->first->end(); ++iter)
    AddHypergraphVertex(sr,iter);

  for(auto iter = sr->first->begin(); iter != sr->first->end(); ++iter)
    for(auto edges = iter->begin(); edges != iter->end(); ++edges)
        AddHypergraphArc(sr,edges);

  return sr;
}

void
CombinedRoadmap::
AddInteractionRoadmap(SemanticRoadmap* _sr) {
  // Check if semantic roadmap already exists
  if(m_interactionRoadmaps.count(_sr))
    return;

  if(!m_cspaceRoadmaps.count(_sr->first)) {
    std::cout << "New cspace roadmap" << std::endl;
  }
  m_cspaceRoadmaps.insert(_sr->first);

  _sr->first->InstallHook(GroupRoadmapType::HookType::AddVertex, 
      "CombinedRoadmap::AddVertex"+std::to_string(m_hookCounter),
      [this, _sr](VI _vi) {
        this->AddHypergraphVertex(_sr,_vi);
  });
  _sr->first->InstallHook(GroupRoadmapType::HookType::AddEdge, 
      "CombinedRoadmap::AddEdge"+std::to_string(m_hookCounter),
      [this, _sr](EI _ei) {
        this->AddHypergraphArc(_sr,_ei);
  });
  _sr->first->InstallHook(GroupRoadmapType::HookType::DeleteVertex, 
      "CombinedRoadmap::DeleteVertex"+std::to_string(m_hookCounter),
      [this, _sr](VI _vi) {

  });
  _sr->first->InstallHook(GroupRoadmapType::HookType::DeleteEdge, 
      "CombinedRoadmap::DeleteEdge"+std::to_string(m_hookCounter),
      [this, _sr](EI _ei) {

  });

  m_hookCounter++;

  m_interactionRoadmaps.insert(_sr);

  // Adding the current content of the interaction roadmap to the Hypergraph.
  for(auto iter = _sr->first->begin(); iter != _sr->first->end(); ++iter)
    AddHypergraphVertex(_sr,iter);

  for(auto iter = _sr->first->begin(); iter != _sr->first->end(); ++iter)
    for(auto edges = iter->begin(); edges != iter->end(); ++edges)
        AddHypergraphArc(_sr,edges);
}

void
CombinedRoadmap::
CheckForGoalState(std::set<size_t> _hvids) {
  
  // Check if any hypergraph vertex induces a satisfying state.

  // Collect composite roadmaps with disjoint coverage of all robots.
  std::vector<CompositeSemanticRoadmap> csrs;
  for(auto hvid : _hvids) {

    // Initialize composite semantic roadmap from output sr.
    TMPVertex vertex = m_hypergraph->GetVertexType(hvid);
    auto sr = vertex.sr;
    CompositeSemanticRoadmap csr = {sr};

    // Fill in the csr to cover all robots in the problem.
    std::set<Robot*> robots; 
    for(auto robot : sr->first->GetGroup()->GetRobots()) {
      robots.insert(robot);
    }

    auto completeGroups = BuildRobotGroups(csr,robots,0);

    // Add complete csr to list.
    for(auto newCsr : completeGroups) {
      csrs.push_back(newCsr);
    } 
  }

  // Check composite roadmaps for satisfying state
  for(auto csr : csrs) {
    ActionUpdate compositeUpdate;
    for(auto sr : csr) {
      compositeUpdate = MergeActionUpdates(compositeUpdate,sr->second);
    }

    auto state = ApplyActionUpdate(m_initialState,compositeUpdate);

    bool satisfying = IsGoalState(state);

    if(!satisfying)
      continue;

    // Connect goal state to sink (assumes hvid = 1).
    std::set<size_t> tail;
    for(auto sr : csr) {
      auto group = sr->first->GetGroup();
      auto rvid = state[group].second;
      auto hvid = m_vertexMap[sr][rvid];
      tail.insert(hvid);
    }

    TMPHyperarc arc;
    arc.semantic = true;

    m_hypergraph->AddHyperarc({1},tail,arc);
  }
}

std::vector<CombinedRoadmap::CompositeSemanticRoadmap>
CombinedRoadmap::
BuildRobotGroups(CompositeSemanticRoadmap _csr, std::set<Robot*> _robots, size_t _offset) {

  // Check if covering all robots.
  bool covered = true;
  auto c = this->GetPlan()->GetCoordinator();
  for(auto& robot : this->GetMPProblem()->GetRobots()) {

    // Dont evaluate the coordinator
    if(robot.get() == c->GetRobot())
      continue;

    if(!_robots.count(robot.get())) {
      covered = false;
      break;
    }
  }

  // If we've created a complete composite semantic roadmap, return it.
  if(covered) {
    return {_csr};
  }
  
  std::vector<CompositeSemanticRoadmap> output;

  auto iter = m_semanticRoadmaps.begin();
  std::advance(iter,_offset);
  auto offset = _offset;

  for(; iter != m_semanticRoadmaps.end(); iter++) {

    auto sr = *iter;
    offset++;

    auto robots = _robots;
    auto csr = _csr;

    // Make sure the semantic roadmap is not already included.
    if(csr.count(sr)) {
      continue;
    }

    // Check if robots are disjoint from composite semantic roadmap.
    bool disjoint = true;
    for(auto robot : sr->first->GetGroup()->GetRobots()) {
      if(robots.count(robot)) {
        disjoint = false;
        break;
      }
      robots.insert(robot);
    }

    if(!disjoint) {
      continue;
    }

    // Add the semantic roadmap to the composite one.
    csr.insert(sr);

    // Recursively build csr by robot groups
    auto recursive = BuildRobotGroups(csr,robots,offset);

    for(auto newCsr : recursive) {
      output.push_back(newCsr);
    }
  }

  return output;
}

CombinedRoadmap::ActionUpdate
CombinedRoadmap::
MergeActionUpdates(ActionUpdate _one, ActionUpdate _two) {

  //TODO:: Test function
  // Find point where updates diverge.
  size_t minLength = std::min(_one.updates.size(),_two.updates.size());

  size_t i;
  for(i = 0; i < minLength; i++) {
    auto one = _one.updates[i];
    auto two = _two.updates[i];
  
    // Check if update differs.
    if(one != two) {
      break;
    }
  }

  // If no differ, return longer sequence
  if(i == minLength) {
    return _one.updates.size() >= _two.updates.size() ? _one : _two; 
  }

  // Gather robots touched by update one.
  std::set<Robot*> touchedByOne;

  for(size_t j = i; j < _one.updates.size(); j++) {
    auto update = _one.updates[j];
    for(auto kv : update.first) {
      for(auto robot : kv.first->GetRobots()) {
        touchedByOne.insert(robot);
      }
    }
  }

  // Check for conflicts and attempt to merge.
  for(size_t j = i; j < _two.updates.size(); j++) {
    auto update = _two.updates[j];
    for(auto kv : update.first) {
      for(auto robot : kv.first->GetRobots()) {
        // Check if overlap.
        if(touchedByOne.count(robot)) {
          // Return empty merge if we find a conflict.
          return ActionUpdate();
        }
      }
    }

    // If no conflict found, merge update.
    _one.updates.push_back(update);
  }

  return _one;
}

CombinedRoadmap::State
CombinedRoadmap::
ApplyActionUpdate(State _initial, ActionUpdate _update) {

  //TODO:: Test function
  for(auto update : _update.updates) {

    // Remove current value of elements in the update.
    auto before = update.first;
    for(auto kv : before) {
      _initial.erase(kv.first);
    }

    // Add value of elements after the update.
    auto after = update.second;
    for(auto kv : after) {
      _initial[kv.first] = kv.second;
    }
  }

  return _initial;
}

bool
CombinedRoadmap::
IsGoalState(State _state) {

  //TODO:: Test function

  // Collect the decomposition.
  auto prob = this->GetMPProblem();
  auto c = this->GetPlan()->GetCoordinator();
  auto decomp = prob->GetDecompositions(c->GetRobot())[0].get();

  // Check if each task is satisfied.
  for(auto st : decomp->GetGroupMotionTasks()) {
    auto mt = st->GetGroupMotionTask();
    std::cout << mt->GetLabel() << std::endl;

    // TODO:: Set task group to appropriate group within the state
    if(!st->IsFixedAssignment()) {
      throw RunTimeException(WHERE) << "Non-fixed assignment not yet handled.";
    }

    auto group = mt->GetRobotGroup();

    // Check if the group exists in the state.
    auto iter = _state.find(group);
    if(iter == _state.end()) {
      return false;
    }

    // Check if the current value of the group satisfies the task
    auto grm = iter->second.first;
    auto rvid = iter->second.second;
    auto gcfg = grm->GetVertex(rvid);

    auto satisfied = mt->EvaluateGoalConstraints(gcfg);
    if(!satisfied)
      return false;

    if(m_debug) {
      std::cout << "Found satisfying state. RVID: "
                << rvid
                << ", GroupCfg: "
                << gcfg.PrettyPrint()
                << std::endl;
    }
  }

  return true;
}


size_t 
CombinedRoadmap::
AddHypergraphVertex(SemanticRoadmap* _sr, VI _vi) {

  auto rvid = _vi->descriptor();

  TMPVertex vertex(rvid,_sr);
  auto hvid = m_hypergraph->AddVertex(vertex);

  m_vertexMap[_sr][rvid] = hvid;

  if(hvid == 878)
    std::cout << "HERE" << std::endl;

  return hvid;
}
    
size_t
CombinedRoadmap::
AddHypergraphArc(SemanticRoadmap* _sr, EI _ei) {
  auto source = _ei->source();
  auto target = _ei->target();

  // Check that both exists in semantic roadmap.
  auto vertices = m_vertexMap[_sr];
  auto iter = vertices.find(source);

  if(iter == vertices.end())
    throw RunTimeException(WHERE) << "Source not included in semantic roadmap.";

  iter = vertices.find(target);
  if(iter == vertices.end())
    throw RunTimeException(WHERE) << "Target not included in semantic roadmap.";

  // Initialize hyperarc
  TMPHyperarc arc;
  arc.semantic = false;
  //arc.glp = _ei->property();

  // Construct hyperarc internal paths
  const auto& edgeDescriptors = _ei->property().GetEdgeDescriptors();
  auto grm = _sr->first;
  auto group = grm->GetGroup();
  auto& robots = group->GetRobots();
  for(size_t i = 0; i < robots.size(); i++) {
    auto eid = edgeDescriptors[i];
    auto roadmap = grm->GetRoadmap(i);
    Path* path = new Path(roadmap);
    std::vector<size_t> vids = {eid.source(),eid.target()};
    *path += vids;
    path->SetTimeSteps(_ei->property().GetTimeSteps());   
 
    // Add path to hyperarc
    arc.paths.push_back(path);
  }

  auto hSource = vertices[source];
  auto hTarget = vertices[target];


  auto hid = m_hypergraph->AddHyperarc({hTarget},{hSource},arc);
  return hid;
}

size_t
CombinedRoadmap::
MoveGroupCfg(GroupCfg& _original, GroupRoadmapType* _newRoadmap) {

  GroupCfg gcfg = _original.SetGroupRoadmap(_newRoadmap);
  return _newRoadmap->AddVertex(gcfg);
}

void
CombinedRoadmap::
MoveGroupEdge(size_t _originalSource, size_t _originalTarget, 
              size_t _newSource, size_t _newTarget, 
              GroupRoadmapType* _originalRoadmap, GroupRoadmapType* _newRoadmap) {
  
  auto group = _newRoadmap->GetGroup();

  const auto originalEdge = _originalRoadmap->GetEdge(_originalSource,_originalTarget);

  GroupLocalPlanType newEdge(_newRoadmap, "", originalEdge.Weight(),
          {_newRoadmap->GetVertex(_newSource),_newRoadmap->GetVertex(_newTarget)});

  for(auto robot : group->GetRobots()) {
    auto edge = *(originalEdge.GetEdge(robot));
    newEdge.SetEdge(robot,std::move(edge));
  }

  _newRoadmap->AddEdge(_newSource,_newTarget,newEdge);
}

void
CombinedRoadmap::
RemapState(State& _state) {

  for(auto& kv : _state) {
    // Extract the state info for the group.
    auto group = kv.first;
    auto grm = kv.second.first;
    auto vid = kv.second.second;
    auto gcfg = grm->GetVertex(vid);

    // Make sure the state info for the group is in the local mpSolution.
    m_mpSolution->AddRobotGroup(group);
    auto newGrm = m_mpSolution->GetGroupRoadmap(group);
    auto newVid = MoveGroupCfg(gcfg,newGrm);
    
    // Update the state to match the local representation.
    kv.second = std::make_pair(newGrm,newVid);
  }
}

CombinedRoadmap::Path*
CombinedRoadmap::
MovePathToMPSolution(Path* _path) {

  auto robot = _path->GetRobot();
  auto pathRoadmap = _path->GetRoadmap();
  auto solutionRoadmap = m_mpSolution->GetRoadmap(robot);

  // Check that the path is not already of the local mp solution.
  if(pathRoadmap == solutionRoadmap)
    return _path;

  const auto pathVIDs = _path->VIDs();
  std::vector<size_t> solutionVIDs(pathVIDs.size());

  // Add vertices
  for(size_t i = 0; i < pathVIDs.size(); i++) {
    //TODO::Update with irving's new path representation
    const auto& cfg = pathRoadmap->GetVertex(pathVIDs[i]);
    auto solutionVID = solutionRoadmap->AddVertex(cfg);
    solutionVIDs[i] = solutionVID;
  }

  // Add edges
  for(size_t i = 1; i < pathVIDs.size(); i++) {
    auto edge = pathRoadmap->GetEdge(pathVIDs[i-1],pathVIDs[i]);
    solutionRoadmap->AddEdge(solutionVIDs[i-1],solutionVIDs[i],edge);
  }

  auto path = new Path(solutionRoadmap);
  *path += solutionVIDs;
  return path;
}

/*----------------------------------------------------------------------------*/

/*istream&
operator>>(istream& _is, TMPVertex& _vertex) {
  return _is;
}*/


ostream&
operator<<(ostream& _os, const CombinedRoadmap::TMPVertex& _vertex) {
  auto rvid = _vertex.rvid;
  auto grm = _vertex.sr->first;
  auto group = grm->GetGroup();
  auto gcfg = grm->GetVertex(rvid);

  _os << "VID: " << rvid << ", RobotGroup: " << group->GetLabel() << std::endl;
  _os << "\t";
  for(auto robot : group->GetRobots()) {
    _os << robot->GetLabel() << " : " << gcfg.GetRobotCfg(robot).PrettyPrint() << std::endl;
  }

  return _os;
}

/*----------------------------------------------------------------------------*/



