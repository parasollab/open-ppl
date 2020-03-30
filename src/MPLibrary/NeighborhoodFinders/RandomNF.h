#ifndef PMPL_RANDOM_NF_H_
#define PMPL_RANDOM_NF_H_

#include "NeighborhoodFinderMethod.h"

#include <set>
#include <string>


////////////////////////////////////////////////////////////////////////////////
/// Selects a set of random neighbors from the roadmap.
///
/// @note This assumes a fairly small k value compared to the candidate set
///       size, and will perform poorly otherwise.
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class RandomNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename RoadmapType::VID            VID;
    typedef typename RoadmapType::VertexSet      VertexSet;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType      GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;
    using typename NeighborhoodFinderMethod<MPTraits>::OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    RandomNF();

    RandomNF(XMLNode& _node);

    virtual ~RandomNF() = default;

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

};

/*------------------------------- Construction -------------------------------*/

template <class MPTraits>
RandomNF<MPTraits>::
RandomNF() : NeighborhoodFinderMethod<MPTraits>(Type::K) {
  this->SetName("RandomNF");
}


template <class MPTraits>
RandomNF<MPTraits>::
RandomNF(XMLNode& _node):
NeighborhoodFinderMethod<MPTraits>(_node, Type::K) {
  this->SetName("RandomNF");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <class MPTraits>
void
RandomNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tk: " << this->m_k
      << std::endl;
}

/*----------------------- NeighborhoodFinder Interface -----------------------*/

template <typename MPTraits>
void
RandomNF<MPTraits>::
FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  this->FindNeighborsImpl(_r, _cfg, _candidates, _out);
}


template <typename MPTraits>
void
RandomNF<MPTraits>::
FindNeighbors(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  this->FindNeighborsImpl(_r, _cfg, _candidates, _out);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
template <typename AbstractRoadmapType>
void
RandomNF<MPTraits>::
FindNeighborsImpl(AbstractRoadmapType* const _r,
    const typename AbstractRoadmapType::CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  VertexSet foundVIDs;
  std::set<Neighbor> closest;

  // Look for up to m_k random neighbors. Try until we find enough or run out of
  // choices.
  while(foundVIDs.size() < _candidates.size() and closest.size() < this->m_k) {
    // Pick a random candidate. Try again if we already found it.
    const VID vid = RandomElement(_candidates);
    if(foundVIDs.count(vid))
      continue;
    foundVIDs.insert(vid);

    // Check for prior connection.
    if(this->m_unconnected and this->DirectEdge(_r, _cfg, vid))
      continue;

    // Check against connection to self.
    const auto& node = _r->GetVertex(vid);
    if(node == _cfg)
      continue;

    // Get the distance from the query cfg to the candidate. If it is infinite,
    // these configurations are not connectable.
    const double distance = dm->Distance(node, _cfg);
    if(std::isinf(distance))
      continue;

    closest.emplace(vid, distance);
  }

  for(const auto& neighbor : closest)
    _out = neighbor;
}

/*----------------------------------------------------------------------------*/

#endif
