#include "RadiusNF.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------ Construction --------------------------------*/

RadiusNF::
RadiusNF() : NeighborhoodFinderMethod(Type::RADIUS) {
  this->SetName("RadiusNF");
}


RadiusNF::
RadiusNF(XMLNode& _node) :
    NeighborhoodFinderMethod(_node, Type::RADIUS) {
  this->SetName("RadiusNF");

  m_useFallback = _node.Read("useFallback", false, m_useFallback,
      "Use nearest node if none are found within the radius.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void
RadiusNF::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod::Print(_os);
  _os << "\tradius: " << this->m_radius
      << "\n\tfallback to nearest: " << m_useFallback
      << std::endl;
}

/*-------------------- NeighborhoodFinderMethod Overrides --------------------*/

void
RadiusNF::
FindNeighbors(RoadmapType* const _r, const Cfg& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  auto dm = this->GetMPLibrary()->GetDistanceMetric(this->m_dmLabel);

  // The neighbor to use as a fallback if m_useFallback is set.
  Neighbor fallback;

  // Find all nodes within radius
  std::multiset<Neighbor> inRadius;

  for(const VID vid : _candidates) {
    // Check for connectedness.
    if(this->m_unconnected and this->DirectEdge(_r, _cfg, vid))
      continue;

    // Check for connection to self.
    const auto& node = _r->GetVertex(vid);
    if(node == _cfg)
      continue;

    // Check distance. If it is infinite, this is not connectable.
    const double distance = dm->Distance(_cfg, node);
    if(std::isinf(distance))
      continue;

    // If within radius, add to list. If not, check for better fallback node.
    if(distance <= this->m_radius)
      inRadius.emplace(vid, distance);
    else if(m_useFallback and distance < fallback.distance) {
      fallback.distance = distance;
      fallback.target = vid;
    }
  }

  // Return fallback if no nodes were found within the radius.
  if(m_useFallback and inRadius.empty())
    inRadius.insert(fallback);

  std::copy(inRadius.begin(), inRadius.end(), _out);
}


void
RadiusNF::
FindNeighbors(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  auto dm = this->GetMPLibrary()->GetDistanceMetric(this->m_dmLabel);

  // The neighbor to use as a fallback if m_useFallback is set.
  Neighbor fallback;

  // Find all nodes within radius
  std::multiset<Neighbor> inRadius;

  for(const VID vid : _candidates) {
    // Check for connectedness.
    if(this->m_unconnected and this->DirectEdge(_r, _cfg, vid))
      continue;

    // Check for connection to self.
    const auto& node = _r->GetVertex(vid);
    if(node == _cfg)
      continue;

    // Check distance. If it is infinite, this is not connectable.
    const double distance = dm->Distance(_cfg, node);
    if(std::isinf(distance))
      continue;

    // If within radius, add to list. If not, check for better fallback node.
    if(distance <= this->m_radius)
      inRadius.emplace(vid, distance);
    else if(m_useFallback and distance < fallback.distance) {
      fallback.distance = distance;
      fallback.target = vid;
    }
  }

  // Return fallback if no nodes were found within the radius.
  if(m_useFallback and inRadius.empty())
    inRadius.insert(fallback);

  std::copy(inRadius.begin(), inRadius.end(), _out);
}

/*----------------------------------------------------------------------------*/
