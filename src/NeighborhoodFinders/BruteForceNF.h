#ifndef BRUTEFORCENF_H_
#define BRUTEFORCENF_H_

#include "NeighborhoodFinderMethod.h"
#include <cmath>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinders
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class BruteForceNF : public NeighborhoodFinderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    BruteForceNF(string _dmLabel = "", bool _unconnected = false, size_t _k = 5) :
      NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected) {
        this->SetName("BruteForceNF");
        this->m_nfType = K;
        this->m_k = _k;
      }

    BruteForceNF(MPProblemType* _problem, XMLNode& _node) :
      NeighborhoodFinderMethod<MPTraits>(_problem, _node) {
        this->SetName("BruteForceNF");
        this->m_nfType = K;
        this->m_k = _node.Read("k", true, 5, 0, MAX_INT, "Number of neighbors to find");
      }

    virtual void Print(ostream& _os) const {
      NeighborhoodFinderMethod<MPTraits>::Print(_os);
      _os << "\tk: " << this->m_k << endl;
    }

    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighbors(RoadmapType* _rmp,
          InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
          const CfgType& _cfg, OutputIterator _out);

    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighborPairs(RoadmapType* _rmp,
          InputIterator _first1, InputIterator _last1,
          InputIterator _first2, InputIterator _last2,
          OutputIterator _out);
};

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
BruteForceNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {

  GraphType* map = _rmp->GetGraph();
  DistanceMetricPointer dmm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);

  if(!this->m_k) {
    for(InputIterator it = _first; it != _last; ++it)
      if(map->GetVertex(it) != _cfg)
        *_out++ = make_pair(_rmp->GetGraph()->GetVID(it),
            dmm->Distance(map->GetVertex(it), _cfg));
    return _out;
  }

  this->IncrementNumQueries();
  this->StartTotalTime();
  this->StartQueryTime();

  // Keep sorted list of k best so far
  priority_queue<pair<VID, double>, vector<pair<VID, double> >, CompareSecond<VID, double> > pq;
  for(InputIterator it = _first; it != _last; it++) {

    if(this->CheckUnconnected(_rmp, _cfg, map->GetVID(it)))
      continue;

    CfgType node = map->GetVertex(it);

    if(node == _cfg) // Don't connect to self
      continue;

    double dist = dmm->Distance(node, _cfg);

    if(std::isinf(dist))
      continue;

    if(pq.size() < this->m_k){
      VID vid = map->GetVID(it);
      pq.push(make_pair(vid, dist));
    }
    // If better than the worst so far, replace worst so far
    else if(dist < pq.top().second) {
      pq.pop();
      VID vid = map->GetVID(it);
      pq.push(make_pair(vid, dist));
    }
  }

  // Transfer k closest to vector, sorted greatest to least dist
  vector<pair<VID, double> > closest;
  while(!pq.empty()){
    closest.push_back(pq.top());
    pq.pop();
  }

  this->EndQueryTime();
  this->EndTotalTime();

  // Reverse order
  return copy(closest.rbegin(), closest.rend(), _out);
}

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
BruteForceNF<MPTraits>::FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {

  GraphType* map = _rmp->GetGraph();
  DistanceMetricPointer dmm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);

  if(!this->m_k){
    for(InputIterator i1 = _first1; i1!=_last1; ++i1)
      for(InputIterator i2 = _first2; i2!=_last2; ++i2)
        if(i1 != i2)
          *_out++ = make_pair(
              make_pair(map->GetVID(i1), map->GetVID(i2)),
              dmm->Distance(map->GetVertex(i1), map->GetVertex(i2)));
    return _out;
  }

  priority_queue<pair<pair<VID, VID>, double>, vector<pair<pair<VID, VID>, double> >,
    CompareSecond<pair<VID, VID>, double> > pq;

  // Find all pairs
  for(InputIterator it1 = _first1; it1 != _last1; it1++) {
    CfgType node1 = map->GetVertex(it1);
    for(InputIterator it2 = _first2; it2 != _last2; it2++) {
      if(*it1 == *it2) // Don't connect to self
        continue;

      CfgType node2 = map->GetVertex(it2);
      double dist = dmm->Distance(node1, node2);
      if(pq.size() < this->m_k){
        VID vid1 = map->GetVID(it1);
        VID vid2 = map->GetVID(it2);
        pq.push(make_pair(make_pair(vid1, vid2), dist));
      // If better than worst so far, replace worst so far
      }else if(dist < pq.top().second) {
        pq.pop();
        VID vid1 = map->GetVID(it1);
        VID vid2 = map->GetVID(it2);
        pq.push(make_pair(make_pair(vid1, vid2), dist));
      }
    }
  }

  // Transfer k closest to vector, sorted greatest to least dist
  vector<pair<pair<VID, VID>, double> > closest;
  while(!pq.empty()) {
    closest.push_back(pq.top());
    pq.pop();
  }
  // Reverse order
  return copy(closest.rbegin(), closest.rend(), _out);
}

#endif
