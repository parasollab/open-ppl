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
    typedef typename RoadmapType::VertexSet     VertexSet;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType     GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;
    using typename NeighborhoodFinderMethod<MPTraits>::OutputIterator;

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
    ///@name NeighborhoodFinderMethod Overrides
    ///@{

    virtual void FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out) override;

    virtual void FindNeighbors(GroupRoadmapType* const _r,
        const GroupCfgType& _cfg, const VertexSet& _candidates,
        OutputIterator _out) override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Templated implementation for both individual and group versions.
    template <typename AbstractRoadmapType>
    void FindNeighborsImpl(AbstractRoadmapType* const _r,
        const typename AbstractRoadmapType::CfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out);

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
RadiusNF() : NeighborhoodFinderMethod<MPTraits>(Type::RADIUS) {
  this->SetName("RadiusNF");
}


template <typename MPTraits>
RadiusNF<MPTraits>::
RadiusNF(XMLNode& _node) :
    NeighborhoodFinderMethod<MPTraits>(_node, Type::RADIUS) {
  this->SetName("RadiusNF");

  m_useFallback = _node.Read("useFallback", false, m_useFallback,
      "Use nearest node if none are found within the radius.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
RadiusNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tradius: " << this->m_radius
      << "\n\tfallback to nearest: " << m_useFallback
      << std::endl;
}

/*-------------------- NeighborhoodFinderMethod Overrides --------------------*/

template <typename MPTraits>
void
RadiusNF<MPTraits>::
FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  this->FindNeighborsImpl(_r, _cfg, _candidates, _out);
}


template <typename MPTraits>
void
RadiusNF<MPTraits>::
FindNeighbors(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  this->FindNeighborsImpl(_r, _cfg, _candidates, _out);
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
template <typename AbstractRoadmapType>
void
RadiusNF<MPTraits>::
FindNeighborsImpl(AbstractRoadmapType* const _r,
    const typename AbstractRoadmapType::CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

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

#endif
