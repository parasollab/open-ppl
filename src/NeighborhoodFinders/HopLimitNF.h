#ifndef HOPLIMITNF_H_
#define HOPLIMITNF_H_

#include "NeighborhoodFinderMethod.h"
#include "graph/algorithms/count_hop_pairs.h"

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

    HopLimitNF(MPProblemType* _problem, XMLNodeReader& _node) :
      NeighborhoodFinderMethod<MPTraits>(_problem, _node) {
        this->SetName("HopLimitNF");
        this->m_nfType = OTHER;
        m_h = _node.numberXMLParameter("hoplimit", true, MAX_INT, 1, MAX_INT, "Hop Limit");
        m_nfLabel = _node.stringXMLParameter("nfLabel", true, "default", "Neighbor Finder Method");
        _node.warnUnrequestedAttributes();
      }
    
    virtual void PrintOptions(ostream& _os) const {
      NeighborhoodFinderMethod<MPTraits>::PrintOptions(_os);
      _os << "\th: " << m_h << endl
        << "\tnfLabel: " << m_nfLabel << endl;
    }

    template<typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(RoadmapType* _rmp, 
          InputIterator _first, InputIterator _last, const CfgType& _cfg, OutputIterator _out);

    // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
    // represent the _kclosest pairs of VIDs between the two ranges.
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
HopLimitNF<MPTraits>::KClosest(RoadmapType* _rmp, InputIterator _first, InputIterator _last, 
    const CfgType& _cfg, OutputIterator _out) {

  this->IncrementNumQueries();
  this->StartTotalTime();
  this->StartQueryTime();

  GraphType* map = _rmp->GetGraph();
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel);
  typename MPProblemType::DistanceMetricPointer dm = nf->GetDMMethod();

  VID v = map->GetVID(_cfg);
  if(v == INVALID_VID) {
    vector<pair<VID, double> > nearestVID;
    nf->KClosest(_rmp, _first, _last, _cfg, back_inserter(nearestVID));
    v = nearestVID[0].first;
  }

  vector<VID> vRes;
  typedef stapl::sequential::map_property_map<typename GraphType::GRAPH, size_t> ColorMap;
  ColorMap hopMap, colorMap;
  hopMap.put(v, 0); 
  stapl::sequential::hops_detail::hops_visitor<typename GraphType::GRAPH> vis(*map, hopMap, m_h, vRes);
  breadth_first_search_early_quit(*map, v, vis, colorMap);

  set<pair<VID, double>, CompareSecond<VID, double>()> closest;
  for(typename vector<VID>::iterator vit = vRes.begin(); vit!=vRes.end(); ++vit){
    closest.push_back(make_pair(*vit, dm->Distance(this->GetMPPRoblem()->GetEnvironment(), _cfg, this->GetVertex(vit))));
  }

  this->EndQueryTime();
  this->EndTotalTime();
  
  return copy(closest.begin(), closest.end(), _out);
}

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator 
HopLimitNF<MPTraits>::FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1, 
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  cerr << "ERROR:: HopLimitNF::FindNeighborPairs is not yet implemented. Exiting." << endl;
  exit(1);
}

#endif
