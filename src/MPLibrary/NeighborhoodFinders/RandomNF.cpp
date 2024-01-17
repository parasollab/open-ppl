#include "RandomNF.h"

#include "MPLibrary/MPLibrary.h"

#include <set>
#include <string>

/*------------------------------- Construction -------------------------------*/

RandomNF::RandomNF() : NeighborhoodFinderMethod(Type::K) {
  this->SetName("RandomNF");
}

RandomNF::RandomNF(XMLNode& _node) : NeighborhoodFinderMethod(_node, Type::K) {
  this->SetName("RandomNF");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void RandomNF::Print(std::ostream& _os) const {
  NeighborhoodFinderMethod::Print(_os);
  _os << "\tk: " << this->m_k << std::endl;
}

/*----------------------- NeighborhoodFinder Interface -----------------------*/

void RandomNF::FindNeighbors(RoadmapType* const _r,
                             const Cfg& _cfg,
                             const VertexSet& _candidates,
                             OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
                 this->GetNameAndLabel() + "::FindNeighbors");

  auto dm = this->GetMPLibrary()->GetDistanceMetric(this->m_dmLabel);

  VertexSet foundVIDs;
  std::set<Neighbor> closest;

  // Look for up to m_k random neighbors. Try until we find enough or run out of
  // choices.
  while (foundVIDs.size() < _candidates.size() and closest.size() < this->m_k) {
    // Pick a random candidate. Try again if we already found it.
    const VID vid = RandomElement(_candidates);
    if (foundVIDs.count(vid))
      continue;
    foundVIDs.insert(vid);

    // Check for prior connection.
    if (this->m_unconnected and this->DirectEdge(_r, _cfg, vid))
      continue;

    // Check against connection to self.
    const auto& node = _r->GetVertex(vid);
    if (node == _cfg)
      continue;

    // Get the distance from the query cfg to the candidate. If it is infinite,
    // these configurations are not connectable.
    const double distance = dm->Distance(node, _cfg);
    if (std::isinf(distance))
      continue;

    closest.emplace(vid, distance);
  }

  for (const auto& neighbor : closest)
    _out = neighbor;
}

void RandomNF::FindNeighbors(GroupRoadmapType* const _r,
                             const GroupCfgType& _cfg,
                             const VertexSet& _candidates,
                             OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
                 this->GetNameAndLabel() + "::FindNeighbors");

  auto dm = this->GetMPLibrary()->GetDistanceMetric(this->m_dmLabel);

  VertexSet foundVIDs;
  std::set<Neighbor> closest;

  // Look for up to m_k random neighbors. Try until we find enough or run out of
  // choices.
  while (foundVIDs.size() < _candidates.size() and closest.size() < this->m_k) {
    // Pick a random candidate. Try again if we already found it.
    const VID vid = RandomElement(_candidates);
    if (foundVIDs.count(vid))
      continue;
    foundVIDs.insert(vid);

    // Check for prior connection.
    if (this->m_unconnected and this->DirectEdge(_r, _cfg, vid))
      continue;

    // Check against connection to self.
    const auto& node = _r->GetVertex(vid);
    if (node == _cfg)
      continue;

    // Get the distance from the query cfg to the candidate. If it is infinite,
    // these configurations are not connectable.
    const double distance = dm->Distance(node, _cfg);
    if (std::isinf(distance))
      continue;

    closest.emplace(vid, distance);
  }

  for (const auto& neighbor : closest)
    _out = neighbor;
}
