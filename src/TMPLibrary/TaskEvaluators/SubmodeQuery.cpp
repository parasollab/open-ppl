#include "SubmodeQuery.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"

#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/StateGraphs/ModeGraph.h"

#include "Utilities/SSSP.h"

/*------------------------------ Construction ------------------------------*/

SubmodeQuery::
SubmodeQuery() {
  this->SetName("SubmodeQuery");
}

SubmodeQuery::
SubmodeQuery(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("SubmodeQuery");

  m_reverseActions = _node.Read("reverseActions",false,m_reverseActions,
        "Flag to allow immedaite reversal of actions in plan.");
}

SubmodeQuery::
~SubmodeQuery() {}

/*------------------------- Task Evlauator Interface -----------------------*/

void
SubmodeQuery::
Initialize() {
  m_previousSolutions.clear();
  m_vertexMap.clear();
  m_partiallyGroundedHyperarcs.clear();
  m_actionExtendedHypergraph = ActionExtendedHypergraph();
}

/*----------------------------- Helper Functions ---------------------------*/

bool
SubmodeQuery::
Run(Plan* _plan) {

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Run");

  if(!_plan)
    _plan = this->GetPlan();

  // Initialize action extended hyperpath from mode graph grounded hyperpath
  ActionExtendedVertex source;
  source.groundedVID = 0;
  auto sourceVID = m_actionExtendedHypergraph.AddVertex(source);
  m_vertexMap[source.groundedVID].insert(sourceVID);

  auto output = HyperpathQuery();

  if(m_debug) {
    std::cout << "ACTION EXTENDED HYPERGRAPH" << std::endl;
    m_actionExtendedHypergraph.Print();
  }

  if(m_goalVID == MAX_INT)
    return false;

  ConvertToPlan(output,_plan);

  return true;
}

SubmodeQuery::ActionHistory
SubmodeQuery::
CombineHistories(size_t _vid, const std::set<size_t>& _pgh, const ActionHistory& _history) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CombineHistories");
  
  auto composite = _history;
  auto newStory = m_actionExtendedHypergraph.GetVertexType(_vid).history;
  auto parentHIDs = m_actionExtendedHypergraph.GetIncomingHyperarcs(_vid);
  if(parentHIDs.size() > 1 and _vid != 1)
    throw RunTimeException(WHERE) << "Unexpected number of incoming hyperarcs.";

  // Combine histories
  for(auto h : newStory) {
    composite.insert(h);
  }

  // Add parent hyperarcs to composite history
  if(!parentHIDs.empty())
    composite.insert(*parentHIDs.begin());


  // Check that there are no conflicts

  std::set<size_t> incoming;
  std::set<size_t> outgoing;

  outgoing.insert(_vid);
  for(auto v : _pgh) {
    outgoing.insert(v);
  }

  for(auto hid : composite) {
    auto hyperarc = m_actionExtendedHypergraph.GetHyperarc(hid);
    // Make sure tail has not been used as outgoing yet
    for(auto vid : hyperarc.tail) {
      if(outgoing.count(vid))
        return {};
      outgoing.insert(vid);
    }

    // Make sure head has not been used as incoming yet
    for(auto vid : hyperarc.head) {
      if(incoming.count(vid))
        return {};
      incoming.insert(vid);
    }
  }

  return composite;
}

void
SubmodeQuery::
ConvertToPlan(const MBTOutput& _output, Plan* _plan) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ConvertToPlan");

  auto mg = dynamic_cast<ModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto& gh = mg->GetGroundedHypergraph();

  auto last = m_goalVID;
 
  HPElem source = std::make_pair(true,0);
  std::set<HPElem> parents = {source};

  auto path = ConstructPath(last,parents,_output);
  path = AddDanglingNodes(path,parents);
  path = OrderPath(path);

  if(m_debug) {
    std::cout << "Full Path" << std::endl;
    std::vector<size_t> vids;
    std::vector<size_t> hids;
    for(auto e : path) {
      if(e.first) {
        std::cout << "v";
        vids.push_back(e.second);
      }
      else {
        std::cout << "h";
        hids.push_back(e.second);
      }

      std::cout << e.second << ", ";
    }

    std::cout << std::endl;

    std::cout << "End points" << std::endl;
    for(auto vid : vids) {
      auto v = m_actionExtendedHypergraph.GetVertexType(vid).groundedVID;
      auto vertex = mg->GetGroundedHypergraph().GetVertexType(v);
      auto grm = vertex.first;
      if(!grm)
        continue;
      auto gvid = vertex.second;
      auto gcfg = grm->GetVertex(gvid);

      std::cout << vid << ": " << grm->GetGroup()->GetLabel() 
                << ": " << gcfg.PrettyPrint() << std::endl;
    }

    std::cout << "Hyperarc costs" << std::endl;
    for(auto hid : hids) {
     
      auto h = m_actionExtendedHypergraph.GetHyperarcType(hid); 
      auto groundedHA = gh.GetHyperarcType(h);
      auto hyperarcWeight = groundedHA.cost;

      std::cout << h << ":" << hyperarcWeight << std::endl;
    }
  }

  // Initialize a decomposition
  auto top = std::shared_ptr<SemanticTask>(new SemanticTask());
  Decomposition* decomp = new Decomposition(top);


  // Map of initial tasks for each group and a flag indicating if it has 
  // been used as a precedence constraint for any tasks yet.
  std::unordered_map<RobotGroup*,std::pair<bool,SemanticTask*>> initialTasks;

  // Create initial tasks for each robot group
  auto hyperarc = m_actionExtendedHypergraph.GetHyperarc(0);
  for(auto vid : hyperarc.head) {
    auto aev = m_actionExtendedHypergraph.GetVertexType(vid);
    auto vertex = gh.GetVertexType(aev.groundedVID);
    auto grm = vertex.first;
    auto group = grm->GetGroup();
    auto gcfg = grm->GetVertex(vertex.second);
    auto task = std::shared_ptr<GroupTask>(new GroupTask(group));
    for(auto robot : group->GetRobots()) {
      auto cfg = gcfg.GetRobotCfg(robot);
      auto start = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,cfg));
      auto goal = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,cfg));
      MPTask t(robot);
      t.SetStartConstraint(std::move(start));
      t.AddGoalConstraint(std::move(goal));
      task->AddTask(t);
    }
    const std::string label = group->GetLabel()+ ":InitialPath"; 
    auto st = new SemanticTask(label,top.get(),decomp,
        SemanticTask::SubtaskRelation::AND,false,true,task);
    initialTasks[group] = std::make_pair(false,st);
  }

  // Save last set of tasks in each hyperarc
  std::unordered_map<size_t,std::vector<SemanticTask*>> hyperarcTaskMap;

  // Convert each hyperarc into a task
  for(auto elem : path) {
    // Skip the vertices
    if(elem.first)
      continue;

    if(m_debug) {
      std::cout << "Creating semantic tasks for hyperarc: " << elem.second << std::endl;
    }

    // Convert hyperarc to semantic tasks

    // Get grounded hypergraph hyperarc
    auto aeh = m_actionExtendedHypergraph.GetHyperarc(elem.second);
    auto hyperarc = gh.GetHyperarcType(aeh.property);


    // Grab tasks of preceeding hyperarcs
    std::vector<SemanticTask*> previousStage;
    // Check each of the head vertices incoming hyperarcs 
    // (should only be one per vertex)
    for(auto vid : aeh.tail) {
      auto incoming = m_actionExtendedHypergraph.GetIncomingHyperarcs(vid);
      if(incoming.empty())
        continue;

      if(incoming.size() > 1)
        throw RunTimeException(WHERE) << "should never have more than one incoming hyperarc.";

      // Add tasks from previous hyperarc's final stage to preceeding task set
      for(auto task : hyperarcTaskMap[*incoming.begin()]) {
        previousStage.push_back(task);
      }
    }

 
    if(m_debug) {
      std::cout << "Initial task dependencies" << std::endl;
      for(auto task : previousStage) {
        std::cout << "\t" << task->GetLabel() << std::endl;
      }
    }

    // Convert to set of sequentially dependent semantic tasks
    auto& taskSet = hyperarc.taskSet;
    size_t counter = 0;
    for(auto stage : taskSet) {

      if(stage.empty()) {
        counter++;
        continue;
      }
  
      std::vector<SemanticTask*> currentStage;
      for(auto groupTask : stage) {
        // Create semantic task
        const std::string label = std::to_string(aeh.property) + ":" 
                            + std::to_string(aeh.hid) + ":"
                            + "stage-" + std::to_string(counter) + ":"
                            + groupTask->GetRobotGroup()->GetLabel() + ":"
                            + groupTask->GetLabel();
        auto task = new SemanticTask(label,top.get(),decomp,
                 SemanticTask::SubtaskRelation::AND,false,true,groupTask);

        if(m_debug) {
          std::cout << "Creating task: " << task->GetLabel() << std::endl;
        }

        for(auto f : hyperarc.taskFormations[groupTask.get()]) {
          task->AddFormation(f);
        }

        currentStage.push_back(task);

        if(m_debug) {
          std::cout << "With dependencies:" << std::endl;
        }

        // Add stage depedencies
        for(auto previous : previousStage) {
          task->AddDependency(previous,SemanticTask::DependencyType::Completion);
          if(m_debug) {
            std::cout << "\t" << previous->GetLabel() << std::endl;
          }
        }

        // Check if group has been given initial dependency
        auto& init = initialTasks[groupTask->GetRobotGroup()];
        if(init.first or !init.second)
          continue;

        // If not, assign the dependency
        task->AddDependency(init.second,SemanticTask::DependencyType::Completion);
        init.first = true;
        if(m_debug) {
          std::cout << "Assign initial dependency: " << init.second->GetLabel() << std::endl;
        }
      }

      // Iterate stage forward
      previousStage = currentStage;
      counter++;
    }

    // Save last stage in hyperarc task map
    hyperarcTaskMap[aeh.hid] = previousStage;
  }

  plan->SetDecomposition(decomp);
  plan->SetCost(_output.weightMap.at(m_goalVID));
}

std::vector<SubmodeQuery::HPElem>
SubmodeQuery::
ConstructPath(size_t _sink, std::set<HPElem>& _parents, const MBTOutput& _mbt) {

  std::vector<HPElem> path;

  HPElem current;
  current.first = true;
  current.second = _sink;

  path.push_back(current);

  while(!_parents.count(current)) {
    _parents.insert(current);
    if(current.first) {
      current.second = _mbt.vertexParentMap.at(current.second);
    }
    else {
      current.second = _mbt.hyperarcParentMap.at(current.second);
    }

    current.first = !current.first;

    path.push_back(current);
  }
    
  _parents.insert(current);
  std::reverse(path.begin(), path.end());

  path = AddBranches(path, _parents, _mbt);

  return path;
}

std::vector<SubmodeQuery::HPElem>
SubmodeQuery::
AddBranches(std::vector<HPElem> _path, std::set<HPElem>& _parents, const MBTOutput& _mbt) {

  std::vector<HPElem> finalPath = _path;

  size_t offset = 0;

  for(size_t i = 0; i < _path.size(); i++) {
    auto elem = _path[i];

    if(elem.first) 
      continue;

    auto arc = m_actionExtendedHypergraph.GetHyperarc(elem.second);

    // Check if tail set is accounted for.
    for(auto vid : arc.tail) {

      HPElem e;
      e.first = true;
      e.second = vid;

      if(!_parents.count(e)) {
        // If tail vertex not in path, compute its branch back to parents.
        auto branch = ConstructPath(vid,_parents,_mbt);

        // Add branch to final path.
        auto iter = finalPath.begin();
        auto branchStart = branch.begin();
        branchStart++;
        finalPath.insert(iter+(i+offset),branchStart,branch.end());

        // Update offset
        offset += (branch.size()-1);
      }
    }
  }

  return finalPath;
}

std::vector<SubmodeQuery::HPElem>
SubmodeQuery::
AddDanglingNodes(std::vector<HPElem> _path, std::set<HPElem>& _parents) {

  std::vector<HPElem> finalPath = _path;

  size_t offset = 0;

  for(size_t i = 0; i < _path.size(); i++) {
    auto elem = _path[i];
    if(elem.first) 
      continue;

    const auto arc = m_actionExtendedHypergraph.GetHyperarc(elem.second);
    
    for(auto vid : arc.head) {
      HPElem e;
      e.first = true;
      e.second = vid;

      if(_parents.count(e))
        continue;

      _parents.insert(e);

      auto iter = finalPath.begin();
      finalPath.insert(iter+(i+offset+1),e);
      
      offset++;
    }
  }

  return finalPath; 
}

std::vector<SubmodeQuery::HPElem>
SubmodeQuery::
OrderPath(std::vector<HPElem> _path) {
  std::set<HPElem> used;

  std::vector<HPElem> ordered = {_path.front()};
  used.insert(_path.front());

  while(ordered.size() != _path.size()) {
    for(auto elem : _path) {
      // Skip vertices
      if(elem.first)
        continue;

      if(used.count(elem))
        continue;

      // Check if entire tail set is in the ordered path
      auto hyperarc = m_actionExtendedHypergraph.GetHyperarc(elem.second);
      bool ready = true;
      for(auto vid : hyperarc.tail) {
        if(!used.count(std::make_pair(true,vid))) {
          ready = false;
          break;
        }
      }

      // Skip if the entire tail set is not in the ordered path
      if(!ready)
        continue;

      ordered.push_back(elem);
      used.insert(elem);

      for(auto vid : hyperarc.head) {
        HPElem ve = std::make_pair(true,vid);
        ordered.push_back(ve);
        used.insert(ve);
      }
    }
  }

  return ordered; 
}

void
SubmodeQuery::
ComputeHeuristicValues() {
  // Run a dijkstra search backwards through hypergraph as if it was a graph

  // Get graph represnetation grounded hypergraph
  auto mg = dynamic_cast<ModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto& gh = mg->GetGroundedHypergraph();
  auto g = gh.GetReverseGraph();

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

  // Save output distances as heuristic values
  m_costToGoMap = output.distance;


  if(m_debug) {
    std::cout << "Cost 2 Go Values" << std::endl;
  }

  for(auto kv : m_costToGoMap) {
    m_maxDistance = std::max(kv.second,m_maxDistance);

    if(m_debug) {
      std::cout << kv.first << " : " << kv.second << std::endl;
    }
  }
}


/*---------------------------- Hyperpath Functions -------------------------*/

MBTOutput
SubmodeQuery::
HyperpathQuery() {

  // Compute heuristic values
  ComputeHeuristicValues();

  // TODO::Change to check start vertex for each goal specified robot/object
  if(m_costToGoMap.at(0) == 0)
    throw RunTimeException(WHERE) << "Start not connected to goal.";

  // Define hyperpath query functors
  SSSHPTerminationCriterion termination(
    [this](size_t& _vid, const MBTOutput& _mbt) {
      return this->HyperpathTermination(_vid,_mbt);
    }
  );

  SSSHPPathWeightFunction<ActionExtendedVertex,size_t> weight(
    [this](const typename ActionExtendedHypergraph::Hyperarc& _hyperarc,
           const std::unordered_map<size_t,double> _weightMap,
           const size_t _target) {
      return this->HyperpathPathWeightFunction(_hyperarc,_weightMap,_target);
    }
  );

  SSSHPForwardStar<ActionExtendedVertex,size_t> forwardStar(
    [this](const size_t& _vid, ActionExtendedHypergraph* _h) {
        // temp debug
        //{
          //auto vertex = _h->GetVertexType(_vid);
          //auto gvid = vertex.groundedVID;
          //if(gvid == 2 or gvid == 4 or gvid == 6 or gvid == 8)
          //  std::cout << "HERE" << std::endl;
          //if(gvid == 3 or gvid == 5 or gvid == 7 or gvid == 9)
          //  std::cout << "OMG!!!! We do actually make progress!" << std::endl;

        //}
      return this->HyperpathForwardStar(_vid,_h);
    }
  );

  SSSHPHeuristic<ActionExtendedVertex,size_t> heuristic(
    [this](const size_t& _target) {
      return this->HyperpathHeuristic(_target);
    }
  );

  m_goalVID = MAX_INT;

  auto output = SBTDijkstra(&m_actionExtendedHypergraph,0,weight,termination,forwardStar,heuristic);

  m_previousSolutions.insert(m_goalVID);

  return output;
}

SSSHPTermination
SubmodeQuery::
HyperpathTermination(const size_t& _vid, const MBTOutput& _mbt) {

  if(m_previousSolutions.count(_vid))
    return SSSHPTermination::EndBranch;

  // Assuming grounded goal is vid 1
  auto groundedVID = m_actionExtendedHypergraph.GetVertexType(_vid).groundedVID;
  if(groundedVID == 1) {
    m_goalVID = _vid;
    return SSSHPTermination::EndSearch;
  }

  return SSSHPTermination::Continue;
}

double
SubmodeQuery::
HyperpathPathWeightFunction(
          const typename ActionExtendedHypergraph::Hyperarc& _hyperarc,
          const std::unordered_map<size_t,double> _weightMap,
          const size_t _target) {

  double hyperarcWeight = 0;

  auto mg = dynamic_cast<ModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto& gh = mg->GetGroundedHypergraph();
  auto groundedHA = gh.GetHyperarcType(_hyperarc.property);
  
  //TODO::DECIDE IF WE WANT THIS
  hyperarcWeight = groundedHA.cost;
  //hyperarcWeight = std::min(1.,groundedHA.cost);

  double tailWeight = 0;

  for(auto vid : _hyperarc.tail) {
    auto cost = _weightMap.at(vid);
    tailWeight = std::max(tailWeight,cost);
  }

  //double newWeight = (tailWeight == 0) ? hyperarcWeight
  //                                     : hyperarcWeight + std::max(0.0,tailWeight - 1);
  //return newWeight;
  return hyperarcWeight + tailWeight;
}

std::set<size_t>
SubmodeQuery::
HyperpathForwardStar(const size_t& _vid, ActionExtendedHypergraph* _h) {
 
  auto mg = dynamic_cast<ModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto& gh = mg->GetGroundedHypergraph();

  auto aev = _h->GetVertexType(_vid);
  auto groundedVID = aev.groundedVID;

  // Check if vid's parent was in the same mode
  // Used for inter mode blocks
  size_t blockedMode = MAX_INT;

  // Extra for blocking reverse actions
  std::vector<size_t> blockedVertices;

  auto vertexMode = mg->GetModeOfGroundedVID(groundedVID);
  auto incoming  = _h->GetIncomingHyperarcs(_vid);
  if(incoming.size() > 0) {
    auto hid = *(incoming.begin());
    auto hyperarc = _h->GetHyperarc(hid);
    auto tail = hyperarc.tail;
    if(tail.size() == 1) {
      auto parent = *(tail.begin());
      auto parentVertex = _h->GetVertexType(parent);
      auto parentMode = mg->GetModeOfGroundedVID(parentVertex.groundedVID);
      if(vertexMode == parentMode)
        blockedMode = vertexMode;
    }

    for(auto parent : tail) {
      auto parentVertex = _h->GetVertexType(parent);
      auto gvid = parentVertex.groundedVID;
      blockedVertices.push_back(gvid);
    }
  }

  //TODO::Adding code to make it monotonic - decide if we want this or not
  auto& modeHypergraph = mg->GetModeHypergraph();
  std::set<size_t> visitedObjectRobotModes;
  for(auto hid : aev.history) {
    auto hyperarc = _h->GetHyperarc(hid);
    for(auto v : hyperarc.tail) {
      auto gvid  =_h->GetVertexType(v).groundedVID;
      auto mvid = mg->GetModeOfGroundedVID(gvid);
      if(mvid == MAX_INT)
        break;

      auto mode = modeHypergraph.GetVertexType(mvid);
      
      //TODO::Change this to look instead at robots with task specificiations
      if(mode->robotGroup->Size() == 1)
        continue;

      for(auto robot : mode->robotGroup->GetRobots()) {
        if(robot->GetMultiBody()->IsPassive()) {
          visitedObjectRobotModes.insert(mvid);
          break;
        }
      }
    }
  }

  std::set<size_t> fullyGroundedHyperarcs;

  // Build partially grounded hyperarcs
  // Grab forward star in grounded hypergraph
  for(auto hid : gh.GetOutgoingHyperarcs(groundedVID)) {

    auto head = gh.GetHyperarc(hid).head;

    //if(head.count(1))
    //  std::cout << "HAS SEEN THE GOAL!!!" << std::endl;

    // TODO::Decide if we want monotonic
    bool monotonic = true;
    for(auto v : head) {
      auto mvid = mg->GetModeOfGroundedVID(v);
      if(visitedObjectRobotModes.count(mvid)) {
        monotonic = false;
        break;
      }
    }

    if(!monotonic)
      continue;

    // If this hyperarc is the reverse of the one that entered the vertex, continue;
    if(blockedVertices.size() == head.size() and !m_reverseActions and monotonic) {
      bool match = true;
      for(auto vid : blockedVertices) {
        if(head.count(vid))
          continue;

        match = false;
      }
  
      if(match)
        continue;
    }

    // If this vertex's parent is in the same submode,
    // ensure that there is not an additional transition within
    // that submode.
    if(head.size() == 1 and blockedMode != MAX_INT) {
      auto groundedHead = *(head.begin());
      auto headModeVID = mg->GetModeOfGroundedVID(groundedHead);
      if(headModeVID == blockedMode)
        continue;
    }

    // Add brand new action extended hyperarc for this vid
    // It will get filled in and check for full grounding in loop
    if(m_partiallyGroundedHyperarcs[hid].empty())
      m_partiallyGroundedHyperarcs[hid].push_back(PartiallyGroundedHyperarc());
  
    // New partially grounded hyperarcs
    std::vector<std::pair<std::set<size_t>,ActionHistory>> newPartiallyGroundedHyperarcs;
  
    // Add vertex to existing partially grounded hyperarcs
    for(auto& pair : m_partiallyGroundedHyperarcs[hid]) {

      auto pgh = pair.first;
      auto history = pair.second;

      // Check if the corresponding grounded vertex is already included
      bool unique = true;
      for(auto v : pgh) {
        if(v == _vid) {
          unique = false;
          break;
        }
      }

      if(!unique) 
        continue;

      // Check if action history is conflicting
      auto compositeHistory = CombineHistories(_vid,pgh,history);
      if(compositeHistory.empty() and _vid != 0)
        continue;

      // If this is a new valid partially grounded hyperarc, add it to the set
      auto newPGH = pgh;
      newPGH.insert(_vid);

      // Check if it is fully grounded
      if(newPGH.size() != gh.GetHyperarc(hid).tail.size()) {
        // If not, add to set and continue
        newPartiallyGroundedHyperarcs.push_back(std::make_pair(newPGH,compositeHistory));
        continue;
      }
      
      // Add fully grounded hyperarc to the action extended hypergraph

      // Construct head set
      std::set<size_t> head;
      
      for(auto v : gh.GetHyperarc(hid).head) {
        ActionExtendedVertex vertex;
        vertex.groundedVID = v;

        //if(v == 1 or v == 0)
        //  std::cout << "OMG!!!! Stuff is broken!" << std::endl;

        //if(v == 3 or v == 5 or v == 7 or v == 9)
        //  std::cout << "OMG!!!! We do actually make progress!" << std::endl;
        //if(v == 1)
        //  std::cout << "HAS ADDED THE HYPERARC TO THE GOAL!!!" << std::endl;


        vertex.history = compositeHistory;
        auto newVID = m_actionExtendedHypergraph.AddVertex(vertex);
        head.insert(newVID);
      }

      // Add hyperarc to action extended hypergraph
      auto newHID = m_actionExtendedHypergraph.AddHyperarc(head,newPGH,hid);
      fullyGroundedHyperarcs.insert(newHID);
    }

    for(auto pgh : newPartiallyGroundedHyperarcs) {
      m_partiallyGroundedHyperarcs[hid].push_back(pgh);
    }

  }

  // Return any fully grounded hyperarcs
  return fullyGroundedHyperarcs;
}

double 
SubmodeQuery::
HyperpathHeuristic(const size_t& _target) {

  auto mg = dynamic_cast<ModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto& gh = mg->GetGroundedHypergraph();

  auto aev = m_actionExtendedHypergraph.GetVertexType(_target);
  auto vid = aev.groundedVID;

  //if(vid == 3 or vid == 5 or vid == 7 or vid == 9)
  //  std::cout << "HEURISTIC ON GOAL VERTICES" << std::endl;
  //if(vid == 1)
  //  std::cout << "HEURISTIC ON GOAL" << std::endl;

  //TODO::Decide if this is what we want
  auto grm = gh.GetVertexType(vid).first;
  if(!grm) {
    m_searchTimeHeuristicMap[_target] = m_costToGoMap[vid];
    return m_costToGoMap[_target];
  }

  // Check if group has a robot
  bool hasRobot = false;
  for(auto robot : grm->GetGroup()->GetRobots()) {
    if(!robot->GetMultiBody()->IsPassive()) {
      hasRobot = true;
      break;
    }
  }

  // Set heuristic to parent - arc length
  auto incomings = m_actionExtendedHypergraph.GetIncomingHyperarcs(_target);
  if(incomings.size() == 0)
    throw RunTimeException(WHERE) << "All targets should have an incoming hyperarc";

  auto incoming = *(incomings.begin());
  auto incomingArc = m_actionExtendedHypergraph.GetHyperarc(incoming);
  auto parent = *(incomingArc.tail.begin());
  auto hyperarc = gh.GetHyperarcType(incomingArc.property);
  // TODO::Adjust for proper trajectory cost
  //double heuristic = m_searchTimeHeuristicMap[parent] - hyperarc.cost;
  double heuristic = m_searchTimeHeuristicMap[parent] - 1;

  // Check if this vertex is entering or exiting a submode 
  auto groundedParentVID = m_actionExtendedHypergraph.GetVertexType(parent).groundedVID;
  auto parentMode = mg->GetModeOfGroundedVID(groundedParentVID);
  auto targetMode = mg->GetModeOfGroundedVID(vid);

  // If parentMode != targetMode, then this is an entry vertex. 
  // Return the heuristic as is
  if(parentMode != targetMode) {
    m_searchTimeHeuristicMap[_target] = heuristic;
    if(hasRobot) 
      return heuristic + 0.1;
    return heuristic;
  }

  // Otherewise, we have an exit vertex.
  // As an exit vertex, we only have one outgoing hyperarc in the action extended hypergraph
  // Compute max heuristic from child nodes

  auto outgoings = gh.GetOutgoingHyperarcs(vid);
  size_t outgoing = MAX_INT;
  
  // Find outgoing hyperarc that exits the mode
  for(auto hid : outgoings) {
    auto ha = gh.GetHyperarc(hid);
    for(auto v : ha.head) {
      auto m = mg->GetModeOfGroundedVID(v);
      if(m != targetMode) {
        outgoing = hid;
        break;
      }
    }
    if(outgoing != MAX_INT)
      break;
  }

  auto outgoingArc = gh.GetHyperarc(outgoing);

  double cost2go = 0;
  for(auto child : outgoingArc.head) {
    cost2go = std::max(cost2go,m_costToGoMap[child]);
  }

  // TODO::Adjust for proper trajectory cost
  //cost2go += 1;
  cost2go += outgoingArc.property.cost;

  heuristic = std::max(heuristic,cost2go);
  m_searchTimeHeuristicMap[_target] = heuristic;
  if(hasRobot) 
    return heuristic + 0.1;
  return heuristic;

  /*
  auto group = grm->GetGroup();
  for(auto& r : group->GetRobots()) {
    if(r->GetMultiBody()->IsPassive()) {
      auto iter = m_heuristicMap.find(vid);
      if(iter == m_heuristicMap.end())
        return m_maxDistance;
      auto value = m_heuristicMap.at(vid);
      if(value == 0.) {
        m_searchTimeHeuristicMap[_target] = value;
        return value;
      }
      else {
        m_searchTimeHeuristicMap[_target] = value;
        return value + 0.1;
      }
    }
  }

  // Heuristic for robot only modes
  // Get heuristic value for parent
  auto incoming = m_actionExtendedHypergraph.GetIncomingHyperarcs(_target);
  if(incoming.size() < 1) {
    m_searchTimeHeuristicMap[_target] = 0;
    return 0.1;
  }

  double value = 0;
  auto hyperarc = m_actionExtendedHypergraph.GetHyperarc(*incoming.begin());
  for(auto vid : hyperarc.tail) {
    auto iter = m_searchTimeHeuristicMap.find(vid);
    if(iter == m_searchTimeHeuristicMap.end())
      throw RunTimeException(WHERE) << "THIS IS AN ISSUE.";

    value = m_searchTimeHeuristicMap[vid];
    break;
  }

  // Remove cost of hyperarc
  
  //TODO::DECIDE IF WE WANT THIS
  //auto groundedHA = gh.GetHyperarcType(hyperarc.property);
  //hyperarcWeight = groundedHA.cost;
  double hyperarcWeight = 1;
  
  m_searchTimeHeuristicMap[_target] = value - hyperarcWeight;

  if(value - hyperarcWeight < 0)
    throw RunTimeException(WHERE) << "THIS IS AN ISSUE.";

  return value - hyperarcWeight + 0.1;

  */

  /*
  auto iter = m_heuristicMap.find(vid);
  if(iter == m_heuristicMap.end())
    return m_maxDistance;
  return m_heuristicMap.at(vid);
  */
}
/*--------------------------------------------------------------------------*/
istream&
operator>>(istream& _is, const SubmodeQuery::ActionExtendedVertex& _vertex) {
  return _is;
}

ostream&
operator<<(ostream& _os, const SubmodeQuery::ActionExtendedVertex& _vertex) {
  return _os;
}
