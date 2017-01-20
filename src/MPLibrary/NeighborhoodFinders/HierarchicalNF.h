#ifndef HIERARCHICAL_NF_H_
#define HIERARCHICAL_NF_H_

#include "NeighborhoodFinderMethod.h"

#include <containers/sequential/graph/algorithms/connected_components.h>

////////////////////////////////////////////////////////////////////////////////
/// TODO
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class HierarchicalNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    HierarchicalNF(string _dmLabel = "") :
      NeighborhoodFinderMethod<MPTraits>(_dmLabel) {
        this->SetName("HierarchicalNF");
        this->m_nfType = OTHER;
      }

    HierarchicalNF(XMLNode& _node) :
      NeighborhoodFinderMethod<MPTraits>(_node) {
        this->SetName("HierarchicalNF");
        this->m_nfType = OTHER;
        m_nfLabel = _node.Read("nfLabel", true, "default", "Neighbor Finder Method1");
	m_nfLabel2 = _node.Read("nfLabel2", true, "default", "Neighbor Finder Method2");
      }

    virtual void Print(ostream& _os) const {
      NeighborhoodFinderMethod<MPTraits>::Print(_os);
      _os  << "\tnfLabel: " << m_nfLabel << endl;
      _os  << "\tnfLabel2: " << m_nfLabel2 << endl;
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
    string m_nfLabel;
    string m_nfLabel2;

};

template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
HierarchicalNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {

  this->IncrementNumQueries();
  this->StartTotalTime();
  this->StartQueryTime();

  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  auto nf2 = this->GetNeighborhoodFinder(m_nfLabel2);

  vector<pair<VID, double> > closest;
  nf->FindNeighbors(_rmp, _first, _last, _fromFullRoadmap, _cfg,
      back_inserter(closest));

  vector<VID> closestVID;
  closestVID.reserve(closest.size());
  for(auto& p : closest)
    closestVID.push_back(p.first);

  nf2->FindNeighbors(_rmp, closestVID.begin(), closestVID.end(), false,
      _cfg, _out);

  this->EndQueryTime();
  this->EndTotalTime();

  return _out;
}

template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
HierarchicalNF<MPTraits>::FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw RunTimeException(WHERE, "FindNeighborPairs is not yet implemented.");
}

#endif
