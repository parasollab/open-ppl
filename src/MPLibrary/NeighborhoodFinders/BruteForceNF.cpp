#include "BruteForceNF.h"

#include "MPLibrary/MPLibrary.h"

#include <queue>

/*------------------------------- Construction -------------------------------*/

BruteForceNF::
BruteForceNF() : NeighborhoodFinderMethod(Type::K) {
  this->SetName("BruteForceNF");
}


BruteForceNF::
BruteForceNF(XMLNode& _node) :
    NeighborhoodFinderMethod(_node, Type::K) {
  this->SetName("BruteForceNF");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void
BruteForceNF::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod::Print(_os);
  _os << "\tk: " << (this->m_k ? std::to_string(this->m_k) : "all")
      << std::endl;
}

/*-------------------- NeighborhoodFinderMethod Overrides --------------------*/

void
BruteForceNF::
FindNeighbors(RoadmapType* const _r, const Cfg& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  this->FindNeighborsImpl(_r, _cfg, _candidates, _out);
}


void
BruteForceNF::
FindNeighbors(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  this->FindNeighborsImpl(_r, _cfg, _candidates, _out);
}

/*--------------------------------- Helpers ----------------------------------*/

void
BruteForceNF::
FindNeighborsImpl(RoadmapType* const _r, const Cfg& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  auto dm = this->GetMPLibrary()->GetDistanceMetric(this->m_dmLabel);

  if(this->m_debug)
    std::cout << "Checking for nearest "
              << (this->m_k ? std::to_string(this->m_k) : "all")
              << " neighbors with dm '" << this->m_dmLabel << "'."
              << "\n\tQuery cfg: " << _cfg.PrettyPrint()
              << std::endl;

  // Keep a max pq of the k best so far (so that we can quickly test and remove
  // the farthest).
  std::priority_queue<Neighbor> pq;

  for(const VID vid : _candidates) {
    // Check for prior connection.
    if(this->m_unconnected and this->DirectEdge(_r, _cfg, vid))
      continue;

    // Get the candidate Cfg and check against connection to self.
    const auto& node = _r->GetVertex(vid);
    if(node == _cfg)
      continue;

    // Get the distance from the query cfg to the candidate. If it is infinite,
    // these configurations are not connectable.
    const double distance = dm->Distance(node, _cfg);
    if(std::isinf(distance))
      continue;

    // Track the closest m_k neighbors.
    if(!this->m_k or pq.size() < this->m_k)
      pq.emplace(vid, distance);
    else if(distance < pq.top().distance) {
      pq.pop();
      pq.emplace(vid, distance);
    }
  }

  if(this->m_debug)
    std::cout << "\tFound " << pq.size() << " neighbors." << std::endl;

  // Write k closest to vector, sorted greatest to least distance.
  std::vector<Neighbor> closest;
  closest.reserve(pq.size());
  while(!pq.empty()) {
    closest.push_back(pq.top());
    pq.pop();
  }

  // Write to output iterator in reverse order.
  std::copy(closest.rbegin(), closest.rend(), _out);
}


void
BruteForceNF::
FindNeighborsImpl(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  auto dm = this->GetMPLibrary()->GetDistanceMetric(this->m_dmLabel);

  if(this->m_debug)
    std::cout << "Checking for nearest "
              << (this->m_k ? std::to_string(this->m_k) : "all")
              << " neighbors with dm '" << this->m_dmLabel << "'."
              << "\n\tQuery cfg: " << _cfg.PrettyPrint()
              << std::endl;

  // Keep a max pq of the k best so far (so that we can quickly test and remove
  // the farthest).
  std::priority_queue<Neighbor> pq;

  for(const VID vid : _candidates) {
    // Check for prior connection.
    if(this->m_unconnected and this->DirectEdge(_r, _cfg, vid))
      continue;

    // Get the candidate Cfg and check against connection to self.
    const auto& node = _r->GetVertex(vid);
    if(node == _cfg)
      continue;

    // Get the distance from the query cfg to the candidate. If it is infinite,
    // these configurations are not connectable.
    const double distance = dm->Distance(node, _cfg);
    if(std::isinf(distance))
      continue;

    // Track the closest m_k neighbors.
    if(!this->m_k or pq.size() < this->m_k)
      pq.emplace(vid, distance);
    else if(distance < pq.top().distance) {
      pq.pop();
      pq.emplace(vid, distance);
    }
  }

  if(this->m_debug)
    std::cout << "\tFound " << pq.size() << " neighbors." << std::endl;

  // Write k closest to vector, sorted greatest to least distance.
  std::vector<Neighbor> closest;
  closest.reserve(pq.size());
  while(!pq.empty()) {
    closest.push_back(pq.top());
    pq.pop();
  }

  // Write to output iterator in reverse order.
  std::copy(closest.rbegin(), closest.rend(), _out);
}

/*----------------------------------------------------------------------------*/
