#ifndef PMPL_RADIUS_NF_H_
#define PMPL_RADIUS_NF_H_

#include "NeighborhoodFinderMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Find all roadmap nodes within a given radius of a query configuration.
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class RadiusNF: public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename RoadmapType::VID           VID;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType     GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;

    ///@}
    ///@name Construction
    ///@{

    RadiusNF();

    RadiusNF(XMLNode& _node);

    virtual ~RadiusNF() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinderMethod Interface
    ///@{

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(RoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(RoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(GroupRoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(GroupRoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    ///@}

  private:

    ///@name Internal State
    ///@{

    /// Use the nearest node if no nodes are found within the radius.
    bool m_useFallback{false};

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
RadiusNF<MPTraits>::
RadiusNF() : NeighborhoodFinderMethod<MPTraits>() {
  this->SetName("RadiusNF");
  this->m_nfType = Type::RADIUS;
}


template <typename MPTraits>
RadiusNF<MPTraits>::
RadiusNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("RadiusNF");
  this->m_nfType = Type::RADIUS;
  this->m_radius = _node.Read("radius", true, 0.5, 0.0, MAX_DBL, "Radius");
  m_useFallback = _node.Read("useFallback", false, false,
      "Use nearest node if none are found within the radius.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
RadiusNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tradius: " << this->m_radius
      << "\n\tuse nearest: " << m_useFallback
      << std::endl;
}

/*------------------- NeighborhoodFinderMethod Functions ---------------------*/

template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
RadiusNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  // The neighbor to use as a fallback if m_useFallback is set.
  Neighbor fallback;

  // Find all nodes within radius
  std::multiset<Neighbor> inRadius;

  for(InputIterator it = _first; it != _last; it++) {
    // Check for connectedness.
    const VID vid = _rmp->GetVID(it);
    if(this->DirectEdge(_rmp, _cfg, vid))
      continue;

    // Check for connection to self.
    const CfgType& node = _rmp->GetVertex(it);
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

  return std::copy(inRadius.begin(), inRadius.end(), _out);
}


template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
RadiusNF<MPTraits>::
FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  // Find all pairs within radius
  std::multiset<Neighbor> inRadius;

  for(InputIterator it1 = _first1; it1 != _last1; it1++) {
    const CfgType& node1 = _rmp->GetVertex(it1);

    for(InputIterator it2 = _first2; it2 != _last2; it2++) {
      // Check for connection to self.
      if(*it1 == *it2)
        continue;

      // Check distance.
      const CfgType& node2 = _rmp->GetVertex(it2);
      const double distance = dm->Distance(node1, node2);
      if(std::isinf(distance))
        continue;

      // If within radius, add to list
      if(distance <= this->m_radius)
        inRadius.emplace(_rmp->GetVID(it1), _rmp->GetVID(it2), distance);
    }
  }

  return std::copy(inRadius.begin(), inRadius.end(), _out);
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
RadiusNF<MPTraits>::
FindNeighbors(GroupRoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
RadiusNF<MPTraits>::
FindNeighborPairs(GroupRoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*----------------------------------------------------------------------------*/

#endif
