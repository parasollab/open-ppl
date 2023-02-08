#include "GreedyHyperpathQuery.h"

#include "TMPLibrary/StateGraphs/GroundedHypergraph.h"
#include "TMPLibrary/StateGraphs/ModeGraph.h"

/*------------------------------ Construction ------------------------------*/

GreedyHyperpathQuery::
GreedyHyperpathQuery() {
  this->SetName("GreedyHyperpathQuery");
}

GreedyHyperpathQuery::
GreedyHyperpathQuery(XMLNode& _node) : SubmodeQuery(_node) {
  this->SetName("GreedyHyperpathQuery");
}

GreedyHyperpathQuery::
~GreedyHyperpathQuery() { }

/*------------------------- Task Evaluator Interface -----------------------*/

void
GreedyHyperpathQuery::
Initialize() {
  SubmodeQuery::Initialize();
  m_historyGraph = std::unique_ptr<HistoryGraph>(new HistoryGraph());
  m_heuristicValues.clear();
}

/*------------------------- Task Evaluator Functions -----------------------*/

bool
GreedyHyperpathQuery::
Run(Plan* _plan) {

  Initialize();

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Run");
  MethodTimer mt_ind(stats,this->GetNameAndLabel() + "::Run_" + std::to_string(counter));
  counter++;

  if(!_plan)
    _plan = this->GetPlan();

  this->ComputeHeuristicValues();

  // Initialize action extended hyperpath from grounded hypergraph
  ActionExtendedVertex source;
  source.groundedVID = 0;
  auto sourceVID = m_actionExtendedHypergraph.AddVertex(source);
  m_vertexMap[source.groundedVID].insert(sourceVID);

  this->m_goalVID = DFS(sourceVID);
   
  if(this->m_goalVID == INVALID_VID)
    return false;


  ConvertToPlan(_plan);

  if(this->m_writeHypergraph) {
    m_actionExtendedHypergraph.Print(this->GetMPProblem()->GetBaseFilename() +
                "-action-extended.hyp");
  }

  return true;

  /*if(this->m_debug) {
    this->m_actionExtendedHypergraph.Print();
    std::cout << std::endl << std::endl << "Transtion History" << std::endl;
    auto history = m_historyGraph->GetVertex(vid);
    for(auto hid : history) {
      std::cout << hid << std::endl;
    }
  }*/
}

/*----------------------------- Helper Functions ---------------------------*/

GreedyHyperpathQuery::VID
GreedyHyperpathQuery::
DFS(const VID _source) {

  m_historyGraph = std::unique_ptr<HistoryGraph>(new HistoryGraph());

  ActionHistory root;
  auto rootVID = m_historyGraph->AddVertex(root);

  std::vector<VID> priority_queue = {rootVID};

  while(!priority_queue.empty()) {
    VID v = priority_queue.back();
    priority_queue.pop_back();

    if(this->m_debug) {
      this->m_actionExtendedHypergraph.Print();
      std::cout << std::endl << std::endl << "Transtion History" << std::endl;
      auto history = m_historyGraph->GetVertex(v);
      for(auto hid : history) {
        std::cout << hid << std::endl;
      }
    }

    auto end = Termination(v);
    if(end != INVALID_VID)
      return end;

    auto frontier = Frontier(v);
    for(auto u : frontier) {
      priority_queue.push_back(u);
    }
  }

  return INVALID_VID;
}


GreedyHyperpathQuery::VID
GreedyHyperpathQuery::
Termination(const VID _vid) {
  auto history = m_historyGraph->GetVertex(_vid);
  if(history.empty())
    return INVALID_VID;

  auto iter = history.end();
  iter--;
  for(;iter != history.begin(); iter--) {
    auto hid = *iter;
    auto hyperarc = m_actionExtendedHypergraph.GetHyperarc(hid);

    for(auto aid : hyperarc.head) {
      auto vertex = this->m_actionExtendedHypergraph.GetVertex(aid);
      auto gid = vertex.property.groundedVID;
      if(gid == 1) {

        // TODO::Put this in its own function
        // Write to the m_mbt to replicate having performed an actualy hyperpath query
        VID sourceVID = 0; // This should be passed in or set somewhere instead of assumed to be 0
        m_mbt.vertexParentMap[sourceVID] = MAX_INT;
        //auto history = this->m_actionExtendedHypergraph.GetVertexType(m_goalVID).history;
        for(auto hid : history) {
          auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);
          for(auto vid : hyperarc.head) {
            m_mbt.vertexParentMap[vid] = hid;
          }

          auto vid = *(hyperarc.tail.begin());
          m_mbt.hyperarcParentMap[hid] = vid;
        }
        //TODO:: Compute the actual cost for this
        m_mbt.weightMap[aid] = 0.1;

        return aid;
      }
    }
  }

  return INVALID_VID;
}

std::vector<GreedyHyperpathQuery::VID>
GreedyHyperpathQuery::
Frontier(const VID _vid) {

  auto history = m_historyGraph->GetVertex(_vid);
  std::set<VID> frontierVIDs;
  std::set<VID> internalVIDs;
  for(auto hid : history) {
    auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);
    for(auto v : hyperarc.head) {
      if(internalVIDs.count(v))
        continue;

      frontierVIDs.insert(v);
    }

    for(auto v : hyperarc.tail) {
      if(frontierVIDs.erase(v))
        continue;

      internalVIDs.insert(v);
    }
  }

  if(history.empty()) {
    frontierVIDs.insert(0);
  }

  std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>> transNeighbors;
  std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>> motionNeighbors;

  auto quantum = BuildQuantumFrontier(frontierVIDs,history);

  for(auto v : quantum) {
    ExpandVertex(_vid,v,quantum,history,transNeighbors,motionNeighbors);
  }

  std::vector<VID> neighbors;
  while(!motionNeighbors.empty()) {
    neighbors.push_back(motionNeighbors.top().second);
    motionNeighbors.pop();
  }

  while(!transNeighbors.empty()) {
    neighbors.push_back(transNeighbors.top().second);
    transNeighbors.pop();
  }

  return neighbors;
}

std::set<GreedyHyperpathQuery::VID>
GreedyHyperpathQuery::
BuildQuantumFrontier(std::set<VID> _frontier, ActionHistory _history) {
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_ghLabel).get());

  // Add all possible motion transitions to the history (ignoring conflicts)
  std::set<VID> newVertices;
  for(auto v : _frontier) {
    auto fs = this->HyperpathForwardStar(v,&(this->m_actionExtendedHypergraph));
    for(auto hid : fs) {
      
      // Check if hyperarc is a motion transition
      auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);

      // Check that this isn't the head isn't the sink vertex
      if(hyperarc.head.size() == 1 and 
         this->m_actionExtendedHypergraph.GetVertexType(*(hyperarc.head.begin())).groundedVID == 1)
        continue;

      // TODO::This is a pretty loose check - should go back to mode 
      //       graph (task space hypergraph) and check if they're the same
      auto groundedHyperarc = gh->GetHyperarc(hyperarc.property);
      if(!(groundedHyperarc.head.size() == 1 and groundedHyperarc.tail.size() == 1)) {
        continue;
      }

      for(auto vid : hyperarc.head) {
        newVertices.insert(vid);
        this->HyperpathForwardStar(vid,&(this->m_actionExtendedHypergraph));
      }

      _history.insert(hid);
    }
  }

  // Compute the new frontier
  for(auto v : newVertices) {
    _frontier.insert(v);
  }

  return _frontier;
}

void
GreedyHyperpathQuery::
ExpandVertex(const VID _source, const VID _vid, const std::set<VID> _frontier, 
             const ActionHistory _history, 
             std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>>& _transNeighbors,
             std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>>& _motionNeighbors) {

  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_ghLabel).get());

  auto fs = this->HyperpathForwardStar(_vid,&(this->m_actionExtendedHypergraph));
  for(auto hid : fs) {

    // Check if hyperarc is a motion transition (and not that the head isn't the sink vertex)
    auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);
    auto groundedHyperarc = gh->GetHyperarc(hyperarc.property);
    // TODO::This is a pretty loose check - should go back to mode 
    //       graph (task space hypergraph) and check if they're the same
    if(groundedHyperarc.head.size() == 1 and groundedHyperarc.tail.size() == 1 and 
       this->m_actionExtendedHypergraph.GetVertexType(*(hyperarc.head.begin())).groundedVID != 1) {
      /*auto newVID = *(hyperarc.head.begin());
      auto front = _frontier;
      front.insert(newVID);
      front.erase(_vid);
      auto hist = _history;
      hist.insert(hid);
      ExpandVertex(_source,newVID,front,hist,_transNeighbors,_motionNeighbors);
      */
      // Already computed possible motion transitions in the quantum frontier
      continue;
    }

    // Check if hyperarc is adjacent to history (all tail vertices along frontier)
    bool adjacent = true;
    for(auto vid : hyperarc.tail) {
      if(!_frontier.count(vid))
        adjacent = false;
    }
    if(!adjacent)
      continue;

    // Create new history
    auto newHistory = _history;
    // Add last hyperarc
    newHistory.insert(hid);

    // Add any predecessor hyperarcs (computed in the quantum frontier)
    for(auto v : hyperarc.tail) {
      for(auto h : this->m_actionExtendedHypergraph.GetIncomingHyperarcs(v)) {
        newHistory.insert(h);
      }
    }

    // Check if hyperarc is compatible with full history
    if(!IsValidHistory(newHistory))
      //if(newHistory.empty() and hid != 0)
      continue;

    // If it is, add the extension to the graph
    if(m_historyGraph->IsVertex(newHistory))
      continue;
    auto vid = m_historyGraph->AddVertex(newHistory);
    m_historyGraph->AddEdge(_source,vid,hid);

    // TODO::Find better way to compute this (with caching)
    double heuristic = 0;
    for(auto vid : hyperarc.tail) {
      heuristic = std::max(heuristic,HyperpathHeuristic(vid));
      break;
    }
    m_heuristicValues[vid] = heuristic;
    // TODO::This is a pretty loose check - should go back to mode 
    //       graph (task space hypergraph) and check if they're the same
    if(hyperarc.head.size() == 1 and hyperarc.tail.size() == 1) {
      _motionNeighbors.emplace(heuristic,vid);
    }
    else {
      _transNeighbors.emplace(heuristic,vid);
    }
  }
}

bool
GreedyHyperpathQuery::
IsValidHistory(const ActionHistory& _history) {
  std::set<VID> outgoing;

  for(auto hid : _history) {
    auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);
    for(auto vid : hyperarc.tail) {
      if(m_hyperarcConstraintTails[hid].count(vid)) {
        continue;
      }
      if(outgoing.count(vid)) {
        return false;
      }

      outgoing.insert(vid);
    }
  }

  return true;
}

/*--------------------------------------------------------------------------*/
