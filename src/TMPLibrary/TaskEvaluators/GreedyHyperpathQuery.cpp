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

}

/*------------------------- Task Evaluator Functions -----------------------*/

bool
GreedyHyperpathQuery::
Run(Plan* _plan) {

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Run");
  MethodTimer mt_ind(stats,this->GetNameAndLabel() + "::Run_" + std::to_string(counter));
  counter++;

  if(!_plan)
    _plan = this->GetPlan();

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

  std::vector<VID> queue = {rootVID};

  while(!queue.empty()) {
    VID v = queue.back();
    queue.pop_back();

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
      queue.push_back(u);
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

  std::vector<VID> neighbors;

  for(auto v : frontierVIDs) {
    auto fs = this->HyperpathForwardStar(v,&(this->m_actionExtendedHypergraph));
    for(auto hid : fs) {

      // Check if hyperarc is adjacent to history (all tail vertices along frontier)
      auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);
      bool adjacent = true;
      for(auto vid : hyperarc.tail) {
        if(!frontierVIDs.count(vid))
          adjacent = false;
      }
      if(!adjacent)
        continue;

      // Check if hyperarc is compatible with full history
      auto newHistory = history;
      newHistory.insert(hid);
      if(!IsValidHistory(newHistory))
      //if(newHistory.empty() and hid != 0)
        continue;

      // If it is, add the extension to the graph
      auto vid = m_historyGraph->AddVertex(newHistory);
      m_historyGraph->AddEdge(_vid,vid,hid);
      neighbors.push_back(vid);
    }
  }

  return neighbors;
}

bool
GreedyHyperpathQuery::
IsValidHistory(const ActionHistory& _history) {
  std::set<VID> outgoing;

  for(auto hid : _history) {
    auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);
    for(auto vid : hyperarc.tail) {
      if(outgoing.count(vid))
        return false;

      outgoing.insert(vid);
    }
  }

  return true;
}

/*--------------------------------------------------------------------------*/
