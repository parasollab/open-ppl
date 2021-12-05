#include "SubmodeQuery.h"

#include "TMPLibrary/StateGraphs/ModeGraph.h"

/*------------------------------ Construction ------------------------------*/

SubmodeQuery::
SubmodeQuery() {
  this->SetName("SubmodeQuery");
}

SubmodeQuery::
SubmodeQuery(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("SubmodeQuery");
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
  
  auto composite = _history;
  auto newStory = m_actionExtendedHypergraph.GetVertexType(_vid).history;
  auto parentHIDs = m_actionExtendedHypergraph.GetIncomingHyperarcs(_vid);
  if(parentHIDs.size() > 1)
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
  // TODO::All of this

  // Backtrace hyperpath from output
 
  auto last = m_goalVID;
 
  HPElem source = std::make_pair(true,0);
  std::set<HPElem> parents = {source};

  auto path = ConstructPath(last,parents,_output);
  path = AddDanglingNodes(path,parents);

  if(m_debug) {
    std::cout << "Full Path" << std::endl;
    for(auto e : path) {
      if(e.first) 
        std::cout << "v";
      else
        std::cout << "h";

      std::cout << e.second << ", ";
    }
    std::cout << std::endl;
  }

  // TODO::Initialize a decomposition

  // Convert each hyperarc into a task
  for(auto elem : path) {
    // Skip the vertices
    if(e.first)
      continue;

    // TODO::Grab set of tasks from ModeGraph

    // TODO::Convert to set of sequentially dependent semantic tasks

    // TODO::Add dependencies from other hyperarcs
  }
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
        offset += branch.size();
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
/*---------------------------- Hyperpath Functions -------------------------*/

MBTOutput
SubmodeQuery::
HyperpathQuery() {

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
      return this->HyperpathForwardStar(_vid,_h);
    }
  );

  m_goalVID = MAX_INT;

  auto output = SBTDijkstra(&m_actionExtendedHypergraph,0,weight,termination,forwardStar);

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
  
  hyperarcWeight = groundedHA.cost;

  double tailWeight = 0;

  for(auto vid : _hyperarc.tail) {
    auto cost = _weightMap.at(vid);
    tailWeight = std::max(tailWeight,cost);
  }

  return hyperarcWeight + tailWeight;
}

std::set<size_t>
SubmodeQuery::
HyperpathForwardStar(const size_t& _vid, ActionExtendedHypergraph* _h) {
  
  auto mg = dynamic_cast<ModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto& gh = mg->GetGroundedHypergraph();

  auto aev = _h->GetVertexType(_vid);
  auto groundedVID = aev.groundedVID;

  std::set<size_t> fullyGroundedHyperarcs;

  // Build partially grounded hyperarcs
  // Grab forward star in grounded hypergraph
  for(auto hid : gh.GetOutgoingHyperarcs(groundedVID)) {

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

/*--------------------------------------------------------------------------*/
