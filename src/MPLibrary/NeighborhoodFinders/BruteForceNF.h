#ifndef PMPL_BRUTE_FORCE_NF_H_
#define PMPL_BRUTE_FORCE_NF_H_

#include "NeighborhoodFinderMethod.h"

#include <queue>


////////////////////////////////////////////////////////////////////////////////
/// Determine the nearest neighbors with a brute force search.
///
/// This method does a direct distance check between the query configuration and
/// each input candidate.
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class BruteForceNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename MPTraits::CfgType           CfgType;
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

    BruteForceNF(std::string _dmLabel = "", bool _unconnected = false,
        size_t _k = 5);

    BruteForceNF(XMLNode& _node);

    virtual ~BruteForceNF();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinderMethod Interface
    ///@{

    template <typename InputIterator>
    void FindNeighbors(RoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    template <typename InputIterator>
    void FindNeighborPairs(RoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    template <typename InputIterator>
    void FindNeighbors(GroupRoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out);

    template <typename InputIterator>
    void FindNeighborPairs(GroupRoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
BruteForceNF<MPTraits>::
BruteForceNF(std::string _dmLabel, bool _unconnected, size_t _k) :
    NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected) {
  this->SetName("BruteForceNF");
  this->m_nfType = Type::K;
  this->m_k = _k;
}


template <typename MPTraits>
BruteForceNF<MPTraits>::
BruteForceNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("BruteForceNF");
  this->m_nfType = Type::K;
  this->m_k = _node.Read("k", true, 5, 0, MAX_INT, "Number of neighbors to find");
}


template <typename MPTraits>
BruteForceNF<MPTraits>::
~BruteForceNF() = default;

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
BruteForceNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tk: " << this->m_k
      << std::endl;
}

/*-------------------- NeighborhoodFinderMethod Interface --------------------*/

template <typename MPTraits>
template <typename InputIterator>
void
BruteForceNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  if(this->m_debug)
    std::cout << "Checking for nearest " << this->m_k
              << " neighbors with dm '" << this->m_dmLabel << "'."
              << std::endl;

  // Keep sorted list of k best so far
  /// @todo See if we can do this more efficiently with a std::set (also avoids
  ///       the extra copy at the end).
  std::priority_queue<Neighbor> pq;

  for(InputIterator it = _first; it != _last; it++) {
    // Get the candidate VID and check for connectedness.
    const VID vid = _rmp->GetVID(it);
    if(this->DirectEdge(_rmp, _cfg, vid))
      continue;

    // Get the candidate Cfg and check against connection to self.
    const CfgType& node = _rmp->GetVertex(it);
    if(node == _cfg)
      continue;

    // Get the distance from the query cfg to the candidate. If it is infinite,
    // these configurations are not connectable.
    const double distance = dm->Distance(node, _cfg);
    if(std::isinf(distance))
      continue;

    // Track the closest m_k neighbors.
    if(pq.size() < this->m_k)
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

  // Reverse order
  std::copy(closest.rbegin(), closest.rend(), _out);
}


template <typename MPTraits>
template <typename InputIterator>
void
BruteForceNF<MPTraits>::
FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2, OutputIterator _out) {
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  if(this->m_debug)
    std::cout << "Checking for nearest " << this->m_k
              << " neighbor pairs with dm '" << this->m_dmLabel << "'."
              << std::endl;

  // Find all pairs
  std::priority_queue<Neighbor> pq;

  for(InputIterator it1 = _first1; it1 != _last1; it1++) {
    // Get the first configuration.
    const CfgType& node1 = _rmp->GetVertex(it1);
    const VID vid1 = _rmp->GetVID(it1);

    // Compare it to everything in the second set.
    for(InputIterator it2 = _first2; it2 != _last2; it2++) {
      // Skip connection to self.
      if(*it1 == *it2)
        continue;

      // Get the second configuration.
      const CfgType& node2 = _rmp->GetVertex(it2);
      const VID vid2 = _rmp->GetVID(it2);

      // Check unconnected.
      if(this->DirectEdge(_rmp, node1, vid2))
        continue;

      // Check distance. If it is infinite, these are not connectable.
      const double distance = dm->Distance(node1, node2);
      if(std::isinf(distance))
        continue;

      // Track the best m_k so far.
      if(pq.size() < this->m_k)
        pq.emplace(vid1, vid2, distance);
      else if(distance < pq.top().distance) {
        pq.pop();
        pq.emplace(vid1, vid2, distance);
      }
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

  // Reverse order
  std::copy(closest.rbegin(), closest.rend(), _out);
}


template <typename MPTraits>
template <typename InputIterator>
void
BruteForceNF<MPTraits>::
FindNeighbors(GroupRoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  if(this->m_debug)
    std::cout << "Checking for nearest " << this->m_k
              << " neighbors with dm '" << this->m_dmLabel << "'."
              << std::endl;

  // Keep sorted list of k best so far
  std::priority_queue<Neighbor> pq;

  for(InputIterator it = _first; it != _last; it++) {
    // Check for connectedness.
    const VID vid = _rmp->GetVID(it);
    if(this->DirectEdge(_rmp, _cfg, vid))
      continue;

    // Get the configuration and check for connection to self.
    const GroupCfgType& node = _rmp->GetVertex(it);
    if(node == _cfg)
      continue;

    // Check distance. If it is infinite, these are not connectable.
    const double distance = dm->Distance(node, _cfg);
    if(std::isinf(distance))
      continue;

    // Track the closest m_k neighbors.
    if(pq.size() < this->m_k) {
      pq.emplace(vid, distance);
    }
    else if(distance < pq.top().distance) {
      pq.pop();
      pq.emplace(vid, distance);
    }
  }

  if(this->m_debug)
    std::cout << "\tFound " << pq.size() << " neighbors." << std::endl;

  // Transfer k closest to vector, sorted greatest to least distance
  std::vector<Neighbor> closest;
  closest.reserve(pq.size());
  while(!pq.empty()) {
    closest.push_back(pq.top());
    pq.pop();
  }

  // Reverse order
  std::copy(closest.rbegin(), closest.rend(), _out);
}


template <typename MPTraits>
template <typename InputIterator>
void
BruteForceNF<MPTraits>::
FindNeighborPairs(GroupRoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2, OutputIterator _out) {
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  if(this->m_debug)
    std::cout << "Checking for nearest " << this->m_k
              << " neighbor pairs with dm '" << this->m_dmLabel << "'."
              << std::endl;

  // Find all pairs
  std::priority_queue<Neighbor> pq;

  for(InputIterator it1 = _first1; it1 != _last1; it1++) {
    // Get the first configuration.
    const GroupCfgType& node1 = _rmp->GetVertex(it1);
    const VID vid1 = _rmp->GetVID(it1);

    // Compare it to everything in the second set.
    for(InputIterator it2 = _first2; it2 != _last2; it2++) {
      // Check for connection to self.
      if(*it1 == *it2)
        continue;

      // Get the second configuration.
      const GroupCfgType& node2 = _rmp->GetVertex(it2);
      const VID vid2 = _rmp->GetVID(it2);

      // Check for connectedness.
      if(this->DirectEdge(_rmp, node1, vid2))
        continue;

      // Check the distance. If it is infinite, these are not connectable.
      const double distance = dm->Distance(node1, node2);
      if(std::isinf(distance))
        continue;

      // Track the closest m_k pairs.
      if(pq.size() < this->m_k)
        pq.emplace(vid1, vid2, distance);
      else if(distance < pq.top().distance) {
        pq.pop();
        pq.emplace(vid1, vid2, distance);
      }
    }
  }

  if(this->m_debug)
    std::cout << "\tFound " << pq.size() << " neighbors." << std::endl;

  // Write k closest to vector, sorted greatest to least distance.
  std::vector<Neighbor> closest;
  while(!pq.empty()) {
    closest.push_back(pq.top());
    pq.pop();
  }

  // Reverse order
  std::copy(closest.rbegin(), closest.rend(), _out);
}

/*----------------------------------------------------------------------------*/

#endif
