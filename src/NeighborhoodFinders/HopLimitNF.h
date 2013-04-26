#ifndef HOPLIMITNF_H_
#define HOPLIMITNF_H_

#include "NeighborhoodFinderMethod.h"
#include "graph/algorithms/count_hop_pairs.h"

using namespace std;

class Environment;

template<class MPTraits>
class HopLimitNF : public NeighborhoodFinderMethod<MPTraits> {
 public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;

    HopLimitNF(MPProblemType* _problem = NULL, string _dmLabel = "", int _h=1, string _label = "") :
      NeighborhoodFinderMethod<MPTraits>(_problem, _dmLabel, _label), m_h(_h) {
        this->SetName("HopLimitNF");
      }

    HopLimitNF(MPProblemType* _problem, XMLNodeReader& _node) :
      NeighborhoodFinderMethod<MPTraits>(_problem, _node) {
        this->SetName("HopLimitNF");
        m_h = _node.numberXMLParameter("hoplimit", true, MAX_INT, 1, MAX_INT, "Hop Limit");
        m_nf = _node.stringXMLParameter("nfLabel", true, "default", "Neighbor Finder Method");
        _node.warnUnrequestedAttributes();
      }

    virtual ~HopLimitNF() {}

    template<typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(RoadmapType* _rmp, 
          InputIterator _first, InputIterator _last, CfgType _cfg, size_t _k, OutputIterator _out);

 private:
   int m_h;
   string m_nf;
};

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator 
HopLimitNF<MPTraits>::KClosest(RoadmapType* _roadmap, InputIterator _first, InputIterator _last, 
        CfgType _cfg, size_t _k, OutputIterator _out) {
  
  vector<VID> vRes, nearestVID;

  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nf);
  nf->KClosest(_roadmap, _first, _last, _cfg, 1, back_inserter(nearestVID));

  stapl::sequential::map_property_map<typename GraphType::GRAPH, size_t> hopMap;
  hopMap.put(nearestVID[0], 0); 
  stapl::sequential::map_property_map<typename GraphType::GRAPH, size_t> colorMap;
  GraphType* rmapG=_roadmap->GetGraph();
  stapl::sequential::hops_detail::hops_visitor<typename GraphType::GRAPH> vis(*rmapG, hopMap, m_h, vRes);
  breadth_first_search_early_quit(*rmapG,nearestVID[0],vis,colorMap);
  nf->KClosest(_roadmap, vRes.begin(), vRes.end(), _cfg, _k, _out);
  return _out;
}

#endif
