#ifndef RANDOMNF_H_
#define RANDOMNF_H_

#include "NeighborhoodFinderMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinders
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class RandomNF : public NeighborhoodFinderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    RandomNF(string _dmLabel = "", bool _unconnected = false, size_t _k = 5):
      NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected) {
        this->SetName("RandomNF");
        this->m_nfType = K;
        this->m_k = _k;
      }

    RandomNF(MPProblemType* _problem, XMLNode& _node):
      NeighborhoodFinderMethod<MPTraits>(_problem,_node) {
        this->SetName("RandomNF");
        this->m_nfType = K;
        this->m_k = _node.Read("k", true, 5, 0, MAX_INT, "Number of neighbors to find");
      }

    virtual void Print(ostream& _os) const {
      NeighborhoodFinderMethod<MPTraits>::Print(_os);
      _os << "\tk: " << this->m_k << endl;
    }

    //Find k-random neighbors.
    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighbors(RoadmapType* _rmp,
          InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
          const CfgType& _cfg, OutputIterator _out);

    //Find k-random pairs of neighbors
    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighborPairs(RoadmapType* _rmp,
          InputIterator _first1, InputIterator _last1,
          InputIterator _first2, InputIterator _last2,
          OutputIterator _out);
};

template <class MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
RandomNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {

  GraphType* map = _rmp->GetGraph();
  DistanceMetricPointer dmm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);

  this->IncrementNumQueries();
  this->StartTotalTime();
  this->StartQueryTime();

  set<VID> vids;
  VID cvid = map->GetVID(_cfg);

  size_t dist = distance(_first, _last);
  for(size_t i = 0; i < this->m_k && i < dist; ++i) {
    VID vid;
    do {
      vid = map->GetVID(_first + LRand() % dist);
    } while(vids.find(vid) != vids.end() && cvid == vid && this->CheckUnconnected(_rmp, _cfg, vid));
    vids.insert(vid);
    *_out++ = make_pair(vid, dmm->Distance(_cfg, map->GetVertex(vid)));
  }

  this->EndQueryTime();
  this->EndTotalTime();

  return _out;
}

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
RandomNF<MPTraits>::FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {

  GraphType* map = _rmp->GetGraph();
  DistanceMetricPointer dmm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);

  set<pair<VID, VID> > ids;

  size_t dist1 = distance(_first1, _last1), dist2 = distance(_first2, _last2);
  for(size_t i=0; i < this->m_k && i < dist1 && i < dist2; ++i) {
    VID vid1, vid2;
    pair<VID, VID> pairId;
    do {
      vid1 = map->GetVID(_first1 + LRand() % dist1);
      vid2 = map->GetVID(_first2 + LRand() % dist2);
      pairId = make_pair(vid1, vid2);
    } while(ids.find(pairId) != ids.end() && vid1 == vid2);
    ids.insert(pairId);
    *_out++ = make_pair(
        make_pair(vid1, vid2),
        dmm->Distance(map->GetVertex(vid1), map->GetVertex(vid2)));
  }

  return _out;
}

#endif
