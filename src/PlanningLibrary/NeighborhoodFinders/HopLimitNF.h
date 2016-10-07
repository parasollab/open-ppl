#ifndef HOPLIMITNF_H_
#define HOPLIMITNF_H_

#include "NeighborhoodFinderMethod.h"

#include <containers/sequential/graph/algorithms/count_hop_pairs.h>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinders
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class HopLimitNF : public NeighborhoodFinderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;

    HopLimitNF(string _dmLabel = "", size_t _h=1) :
      NeighborhoodFinderMethod<MPTraits>(_dmLabel), m_h(_h) {
        this->SetName("HopLimitNF");
        this->m_nfType = OTHER;
      }

    HopLimitNF(MPProblemType* _problem, XMLNode& _node) :
      NeighborhoodFinderMethod<MPTraits>(_problem, _node) {
        this->SetName("HopLimitNF");
        this->m_nfType = OTHER;
        m_h = _node.Read("hoplimit", true, MAX_INT, 1, MAX_INT, "Hop Limit");
        m_nfLabel = _node.Read("nfLabel", true, "default", "Neighbor Finder Method");
      }

    virtual void Print(ostream& _os) const {
      NeighborhoodFinderMethod<MPTraits>::Print(_os);
      _os << "\th: " << m_h << endl
        << "\tnfLabel: " << m_nfLabel << endl;
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

  private:
    size_t m_h;
    string m_nfLabel;
};

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
HopLimitNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  #ifndef _PARALLEL // proper fix will be to call parallel versions of graph algorithms called here
  this->IncrementNumQueries();
  this->StartTotalTime();
  this->StartQueryTime();

  GraphType* graph = _rmp->GetGraph();
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel);
  typename MPProblemType::DistanceMetricPointer dm = nf->GetDMMethod();

  VID v = graph->GetVID(_cfg);
  typename GraphType::vertex_iterator vi = graph->find_vertex(v);
  VID parent = vi->property().GetStat("Parent");

  vector<VID> vRes;
  typedef stapl::sequential::map_property_map<typename GraphType::GRAPH, size_t> ColorMap;
  ColorMap hopMap, colorMap;
  hopMap.put(parent, 0);
  stapl::sequential::hops_detail::hops_visitor<typename GraphType::GRAPH> vis(*graph, hopMap, m_h, vRes);
  breadth_first_search_early_quit(*graph, parent, vis, colorMap);
  nf->FindNeighbors(_rmp, vRes.begin(), vRes.end(),
      vRes.size() == graph->get_num_vertices(), _cfg, _out);

  this->EndQueryTime();
  this->EndTotalTime();
  #endif
  return _out;
}

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
HopLimitNF<MPTraits>::FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw RunTimeException(WHERE, "FindNeighborPairs is not yet implemented.");
}

#endif
