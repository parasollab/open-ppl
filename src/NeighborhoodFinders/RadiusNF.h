#ifndef RADIUSNF_H_
#define RADIUSNF_H_

#include "NeighborhoodFinderMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinders
/// @brief Find neighbors within a certain radius.
/// @tparam MPTraits Motion planning universe
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class RadiusNF: public NeighborhoodFinderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    RadiusNF(string _dmLabel = "", bool _unconnected = false, double _r = 1.0) :
      NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected) {
        this->SetName("RadiusNF");
        this->m_nfType = RADIUS;
        this->m_radius = _r;
      }

    RadiusNF(MPProblemType* _problem, XMLNode& _node) :
      NeighborhoodFinderMethod<MPTraits>(_problem, _node) {
        this->SetName("RadiusNF");
        this->m_nfType = RADIUS;
        this->m_radius = _node.Read("radius", true, 0.5, 0.0, MAX_DBL, "Radius");
      }

    virtual void Print(ostream& _os) const {
      NeighborhoodFinderMethod<MPTraits>::Print(_os);
      _os << "\tradius: " << this->m_radius << endl;
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
RadiusNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {

  this->IncrementNumQueries();
  this->StartTotalTime();
  this->StartQueryTime();

  GraphType* map = _rmp->GetGraph();
  DistanceMetricPointer dmm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);
  set<pair<VID, double>, CompareSecond<VID, double> > inRadius;

  // Find all nodes within radius
  for(InputIterator it = _first; it != _last; it++) {

    if(this->CheckUnconnected(_rmp, _cfg, map->GetVID(it)))
      continue;

    CfgType node = map->GetVertex(it);
    if(node == _cfg) // Don't connect to itself
      continue;

    // If within radius, add to list
    double dist = dmm->Distance(_cfg, node);
    if(dist <= this->m_radius)
      inRadius.insert(make_pair(map->GetVID(it), dist));
  }

  this->EndQueryTime();
  this->EndTotalTime();

  return copy(inRadius.begin(), inRadius.end(), _out);
}

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
RadiusNF<MPTraits>::FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {

  GraphType* map = _rmp->GetGraph();
  DistanceMetricPointer dmm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);
  set<pair<pair<VID, VID >, double> > inRadius;

  // Find all pairs within radius
  for(InputIterator it1 = _first1; it1 != _last1; it1++) {
    CfgType node1 = map->GetVertex(it1);
    for(InputIterator it2 = _first2; it2 != _last2; it2++) {
      if(*it1 == *it2) // Don't connect to itself
        continue;
      CfgType node2 = map->GetVertex(it2);

      // If within radius, add to list
      double dist = dmm->Distance(node1, node2);
      if(dist <= this->m_radius){
        inRadius.insert(make_pair(
              make_pair(map->GetVID(it1), map->GetVID(it2)),
              dist));
      }
    }
  }

  return copy(inRadius.begin(), inRadius.end(), _out);
}

#endif
