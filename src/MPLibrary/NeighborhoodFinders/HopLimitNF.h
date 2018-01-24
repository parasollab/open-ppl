#ifndef HOP_LIMIT_NF_H_
#define HOP_LIMIT_NF_H_

#include "NeighborhoodFinderMethod.h"

#include <containers/sequential/graph/algorithms/count_hop_pairs.h>


////////////////////////////////////////////////////////////////////////////////
/// Finds all connected neighbors within some maximum number of hops, and then
/// orders those according to a second NeighborhoodFinder (the 'underlying' NF).
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class HopLimitNF : public NeighborhoodFinderMethod<MPTraits> {

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

    HopLimitNF(const std::string& _dmLabel = "", const size_t _h = 1);

    HopLimitNF(XMLNode& _node);

    virtual ~HopLimitNF();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

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

    size_t m_h;            ///< Maximum number of hops.
    std::string m_nfLabel; ///< Underlying NF.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
HopLimitNF<MPTraits>::
HopLimitNF(const std::string& _dmLabel, const size_t _h)
  : NeighborhoodFinderMethod<MPTraits>(_dmLabel), m_h(_h) {
  this->SetName("HopLimitNF");
  this->m_nfType = OTHER;
}


template <typename MPTraits>
HopLimitNF<MPTraits>::
HopLimitNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("HopLimitNF");
  this->m_nfType = OTHER;
  m_h = _node.Read("hoplimit", true, MAX_INT, 1, MAX_INT, "Hop Limit");
  m_nfLabel = _node.Read("nfLabel", true, "default", "Neighbor Finder Method");
}


template <typename MPTraits>
HopLimitNF<MPTraits>::
~HopLimitNF() = default;

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
HopLimitNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\th: " << m_h
      << "\n\tnfLabel: " << m_nfLabel
      << std::endl;
}

/*-------------------- NeighborhoodFinderMethod Interface --------------------*/

template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
HopLimitNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
#ifdef _PARALLEL
  // Proper fix will be to call parallel versions of graph algorithms called here
#error "HopLimitNF is not implemented for parallel. "\
       "You must either implement it or remove this NF from your traits file."
#endif
  MethodTimer mt(this->GetStatClass(), "HopLimitNF::FindNeighbors");
  this->IncrementNumQueries();

  GraphType* graph = _rmp->GetGraph();
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  auto dm = this->GetDistanceMetric(nf->GetDMLabel());

  VID v = graph->GetVID(_cfg);
  typename GraphType::vertex_iterator vi = graph->find_vertex(v);
  VID parent = vi->property().GetStat("Parent");

  vector<VID> vRes;
  typedef stapl::sequential::map_property_map<typename GraphType::STAPLGraph,
      size_t> ColorMap;
  ColorMap hopMap, colorMap;
  hopMap.put(parent, 0);
  stapl::sequential::hops_detail::hops_visitor<typename GraphType::STAPLGraph>
      vis(*graph, hopMap, m_h, vRes);
  breadth_first_search_early_quit(*graph, parent, vis, colorMap);
  nf->FindNeighbors(_rmp, vRes.begin(), vRes.end(),
      vRes.size() == graph->get_num_vertices(), _cfg, _out);

  return _out;
}


template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
HopLimitNF<MPTraits>::FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw RunTimeException(WHERE, "FindNeighborPairs is not yet implemented.");
}

/*----------------------------------------------------------------------------*/

#endif
