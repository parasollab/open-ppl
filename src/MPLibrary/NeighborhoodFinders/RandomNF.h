#ifndef PMPL_RANDOM_NF_H_
#define PMPL_RANDOM_NF_H_

#include "NeighborhoodFinderMethod.h"

#include <string>
#include <unordered_set>


////////////////////////////////////////////////////////////////////////////////
/// Selects a set of random neighbors from the roadmap.
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

    virtual ~RandomNF();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinder Interface
    ///@{

    template <typename InputIterator>
    void FindNeighbors(RoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    template <typename InputIterator>
    void FindNeighbors(GroupRoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out);

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <class MPTraits>
RandomNF<MPTraits>::
RandomNF() : NeighborhoodFinderMethod<MPTraits>() {
  this->SetName("RandomNF");
  this->m_nfType = Type::K;
}


template <class MPTraits>
RandomNF<MPTraits>::
RandomNF(XMLNode& _node):
NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("RandomNF");
  this->m_nfType = Type::K;
  this->m_k = _node.Read("k", true, 5, 0, MAX_INT, "Number of neighbors to find");
}


template <class MPTraits>
RandomNF<MPTraits>::
~RandomNF() = default;

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

template <class MPTraits>
template <typename InputIterator>
void
RandomNF<MPTraits>::
FindNeighbors(RoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  auto g = _r;
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  std::unordered_set<VID> foundVIDs;
  const VID queryVID = g->GetVID(_cfg);

  const size_t inputSize = std::distance(_first, _last);

  // Look for up to m_k random neighbors.
  for(size_t i = 0; i < this->m_k && i < inputSize; ++i) {
    // Try until we find a valid neighbor or run out of choices.
    while(foundVIDs.size() < inputSize) {
      auto iter = _first;
      std::advance(iter, LRand() % inputSize);
      const VID vid = g->GetVID(iter);

      // Check for invalid conditions.
      const bool alreadyFound = foundVIDs.count(vid),
                 isQuery = queryVID == vid,
                 isConnected = this->DirectEdge(g, _cfg, vid);

      // Track this VID.
      foundVIDs.insert(vid);

      if(alreadyFound or isQuery or isConnected)
        continue;

      // Check distance.
      const double distance = dm->Distance(_cfg, g->GetVertex(vid));
      if(std::isinf(distance))
        continue;

      *_out++ = Neighbor(vid, distance);
      break;
    }
  }
}


template <typename MPTraits>
template <typename InputIterator>
void
RandomNF<MPTraits>::
FindNeighbors(GroupRoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*----------------------------------------------------------------------------*/

#endif
