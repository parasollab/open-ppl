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

    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename RoadmapType::VID            VID;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType      GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;

    ///@}
    ///@name Construction
    ///@{

    HopLimitNF();

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
    OutputIterator FindNeighbors(RoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(RoadmapType* _r,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(GroupRoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(GroupRoadmapType* _r,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    ///@}

  private:

    ///@name Internal State
    ///@{

    size_t m_h{1};         ///< Maximum number of hops.
    std::string m_nfLabel; ///< Underlying NF.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
HopLimitNF<MPTraits>::
HopLimitNF() : NeighborhoodFinderMethod<MPTraits>() {
  this->SetName("HopLimitNF");
  this->m_nfType = Type::OTHER;
}


template <typename MPTraits>
HopLimitNF<MPTraits>::
HopLimitNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("HopLimitNF");
  this->m_nfType = Type::OTHER;

  m_h = _node.Read("hoplimit", true, m_h, size_t(1),
      std::numeric_limits<size_t>::max(), "Hop Limit");

  m_nfLabel = _node.Read("nfLabel", true, "", "Neighbor Finder Method");
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
FindNeighbors(RoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  auto dm = this->GetDistanceMetric(nf->GetDMLabel());

  VID v = _r->GetVID(_cfg);
  typename RoadmapType::vertex_iterator vi = _r->find_vertex(v);
  VID parent = vi->property().GetStat("Parent");

  vector<VID> vRes;

  typedef stapl::sequential::map_property_map<typename RoadmapType::STAPLGraph,
      size_t> ColorMap;
  ColorMap hopMap, colorMap;
  hopMap.put(parent, 0);

  stapl::sequential::hops_detail::hops_visitor<typename RoadmapType::STAPLGraph>
      vis(*_r, hopMap, m_h, vRes);
  breadth_first_search_early_quit(*_r, parent, vis, colorMap);

  nf->FindNeighbors(_r, vRes.begin(), vRes.end(),
      vRes.size() == _r->get_num_vertices(), _cfg, _out);

  return _out;
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
HopLimitNF<MPTraits>::
FindNeighborPairs(RoadmapType* _r,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
HopLimitNF<MPTraits>::
FindNeighbors(GroupRoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
HopLimitNF<MPTraits>::
FindNeighborPairs(GroupRoadmapType* _r,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*----------------------------------------------------------------------------*/

#endif
