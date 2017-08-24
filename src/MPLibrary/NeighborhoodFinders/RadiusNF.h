#ifndef RADIUS_NF_H_
#define RADIUS_NF_H_

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

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a radius neighborhood finder.
    /// @param _dmLabel The distance metric to use.
    /// @param _unconnected Return only neighbors that are not connected?
    /// @param _r Return all nodes within this distance of the query point.
    /// @param _useFallback Return the nearest node if none are in the radius?
    RadiusNF(const std::string& _dmLabel = "", const bool _unconnected = false,
        const double _r = 1.0, const bool _useFallback = false);

    /// Construct a radius neighborhood finder from an XML node.
    /// @param _node The XML node.
    RadiusNF(XMLNode& _node);

    virtual ~RadiusNF() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinderMethod Interface
    ///@{

    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(RoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(RoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    ///@}

  private:

    ///@name Internal State
    ///@{

    bool m_useFallback{false}; ///< Use the nearest node if no nodes are found
                               ///< within the radius.

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
RadiusNF<MPTraits>::
RadiusNF(const std::string& _dmLabel, const bool _unconnected,
    const double _r, const bool _useFallback) :
    NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected) {
  this->SetName("RadiusNF");
  this->m_nfType = RADIUS;
  this->m_radius = _r;
  m_useFallback = _useFallback;
}


template <typename MPTraits>
RadiusNF<MPTraits>::
RadiusNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("RadiusNF");
  this->m_nfType = RADIUS;
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

  this->IncrementNumQueries();
  this->StartTotalTime();
  this->StartQueryTime();

  GraphType* map = _rmp->GetGraph();
  auto dmm = this->GetDistanceMetric(this->m_dmLabel);
  std::multiset<pair<VID, double>, CompareSecond<VID, double> > inRadius;

  // The node used as a fallback if m_useFallback is set. Pair of VID and
  // distance from _cfg. This is returned if no nodes are found within the
  // radius.
  pair<VID, double> fallback;

  // Distance used when tracking the fallback node. Will always be larger than
  // the radius.
  double fallbackDist = std::numeric_limits<double>::max();

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
    // Track nearest node outside of radius and its distance.
    else if(m_useFallback and dist < fallbackDist) {
      fallbackDist = dist;
      fallback = make_pair(map->GetVID(it), fallbackDist);
    }
  }

  this->EndQueryTime();
  this->EndTotalTime();

  // Return fallback if no nodes were found within the radius.
  if(m_useFallback and inRadius.empty())
    inRadius.insert(fallback);

  return copy(inRadius.begin(), inRadius.end(), _out);
}


template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
RadiusNF<MPTraits>::FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {

  GraphType* map = _rmp->GetGraph();
  auto dmm = this->GetDistanceMetric(this->m_dmLabel);
  std::multiset<pair<pair<VID, VID >, double> > inRadius;

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

/*----------------------------------------------------------------------------*/

#endif
