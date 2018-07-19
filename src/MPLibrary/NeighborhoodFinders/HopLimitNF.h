#ifndef PMPL_HOP_LIMIT_NF_H_
#define PMPL_HOP_LIMIT_NF_H_

#include "NeighborhoodFinderMethod.h"

#include <containers/sequential/graph/algorithms/count_hop_pairs.h>


////////////////////////////////////////////////////////////////////////////////
/// Finds all connected neighbors within some maximum number of hops, and then
/// orders those according to a second NeighborhoodFinder (the 'underlying' NF).
/// @todo Adjust this class so that it only finds neighbors within the hop
///       limit. There should be no 'underlying' NF - this can always be
///       achieved with a hierarchical NF.
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
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

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

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(RoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(RoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(GroupRoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(GroupRoadmapType* _rmp,
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
template <typename InputIterator, typename OutputIterator>
OutputIterator
HopLimitNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  GraphType* g = _rmp->GetGraph();
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  auto dm = this->GetDistanceMetric(nf->GetDMLabel());

  VID v = g->GetVID(_cfg);
  typename GraphType::vertex_iterator vi = g->find_vertex(v);
  VID parent = vi->property().GetStat("Parent");

  vector<VID> vRes;

  typedef stapl::sequential::map_property_map<typename GraphType::STAPLGraph,
      size_t> ColorMap;
  ColorMap hopMap, colorMap;
  hopMap.put(parent, 0);

  stapl::sequential::hops_detail::hops_visitor<typename GraphType::STAPLGraph>
      vis(*g, hopMap, m_h, vRes);
  breadth_first_search_early_quit(*g, parent, vis, colorMap);

  nf->FindNeighbors(_rmp, vRes.begin(), vRes.end(),
      vRes.size() == g->get_num_vertices(), _cfg, _out);

  return _out;
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
HopLimitNF<MPTraits>::
FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
HopLimitNF<MPTraits>::
FindNeighbors(GroupRoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
HopLimitNF<MPTraits>::
FindNeighborPairs(GroupRoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*----------------------------------------------------------------------------*/

#endif
