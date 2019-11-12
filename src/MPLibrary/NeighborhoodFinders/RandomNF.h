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
    void FindNeighborPairs(RoadmapType* _r,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    template <typename InputIterator>
    void FindNeighbors(GroupRoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out);

    template <typename InputIterator>
    void FindNeighborPairs(GroupRoadmapType* _r,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

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
FindNeighborPairs(RoadmapType* _r,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  auto g = _r;
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  std::set<std::pair<VID, VID>> foundPairs;

  const size_t inputSize1 = std::distance(_first1, _last1),
               inputSize2 = std::distance(_first2, _last2);

  for(size_t i = 0; i < this->m_k and i < inputSize1 and i < inputSize2; ++i) {
    // Try until we find a valid neighbor or run out of choices.
    while(foundPairs.size() < inputSize1 * inputSize2) {
      const VID vid1 = g->GetVID(_first1 + LRand() % inputSize1),
                vid2 = g->GetVID(_first2 + LRand() % inputSize2);
      const CfgType& cfg1 = g->GetVertex(vid1);

      // Check for invalid conditions.
      const bool alreadyFound = foundPairs.count({vid1, vid2}),
                 alreadyConnected = this->DirectEdge(g, cfg1, vid2);

      if(alreadyFound or alreadyConnected)
        continue;

      // Track this pair.
      foundPairs.emplace(vid1, vid2);

      // Check distance.
      const double distance = dm->Distance(cfg1, g->GetVertex(vid2));
      if(std::isinf(distance))
        continue;

      *_out++ = Neighbor(vid1, vid2, distance);
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


template <typename MPTraits>
template <typename InputIterator>
void
RandomNF<MPTraits>::
FindNeighborPairs(GroupRoadmapType* _r,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*----------------------------------------------------------------------------*/

#endif
