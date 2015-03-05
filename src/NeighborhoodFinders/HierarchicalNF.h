#ifndef HIERARCHICALNF_H_
#define HIERARCHICALNF_H_

#include "NeighborhoodFinderMethod.h"

#include <containers/sequential/graph/algorithms/connected_components.h>

template<class MPTraits>
class HierarchicalNF : public NeighborhoodFinderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;

    HierarchicalNF(string _dmLabel = "") :
      NeighborhoodFinderMethod<MPTraits>(_dmLabel) {
        this->SetName("HierarchicalNF");
        this->m_nfType = OTHER;
      }

    HierarchicalNF(MPProblemType* _problem, XMLNodeReader& _node) :
      NeighborhoodFinderMethod<MPTraits>(_problem, _node) {
        this->SetName("HierarchicalNF");
        this->m_nfType = OTHER;
        m_nfLabel = _node.stringXMLParameter("nfLabel", true, "default", "Neighbor Finder Method1");
	m_nfLabel2 = _node.stringXMLParameter("nfLabel2", true, "default", "Neighbor Finder Method2");
        _node.warnUnrequestedAttributes();
      }

    virtual void Print(ostream& _os) const {
      NeighborhoodFinderMethod<MPTraits>::Print(_os);
      _os  << "\tnfLabel: " << m_nfLabel << endl;
      _os  << "\tnfLabel2: " << m_nfLabel2 << endl;
    }

    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighbors(RoadmapType* _rmp,
          InputIterator _first, InputIterator _last, const CfgType& _cfg, OutputIterator _out);

    // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
    // represent the _kclosest pairs of VIDs between the two ranges.
    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighborPairs(RoadmapType* _rmp,
          InputIterator _first1, InputIterator _last1,
          InputIterator _first2, InputIterator _last2,
          OutputIterator _out);

  private:
    string m_nfLabel;
    string m_nfLabel2;
    vector<pair<VID, double> > m_closest;
    vector<VID> ccVIDs;

};

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
HierarchicalNF<MPTraits>::FindNeighbors(RoadmapType* _rmp, InputIterator _first, InputIterator _last,
    const CfgType& _cfg, OutputIterator _out) {

  this->IncrementNumQueries();
  this->StartTotalTime();
  this->StartQueryTime();

  GraphType* graph = _rmp->GetGraph();
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel);
  NeighborhoodFinderPointer nf2 = this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel2);


  nf->FindNeighbors(_rmp, _first, _last, _cfg, back_inserter(m_closest));



 typename vector<pair<VID, double> >::const_iterator ccIt;
    stapl::sequential::vector_property_map<GraphType, size_t> cmap;
    for(ccIt = m_closest.begin(); ccIt != m_closest.end(); ccIt++) {
      cmap.reset();
      ccVIDs.clear();

    get_cc(*graph, cmap, ccIt->first, ccVIDs);
    }

  nf2->FindNeighbors(_rmp,ccVIDs.begin(),ccVIDs.end(), _cfg, _out);

  if(this->m_debug){
    nf->Print(cout);
    nf2->Print(cout);
   }

  this->EndQueryTime();
  this->EndTotalTime();

  return _out;
}

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
HierarchicalNF<MPTraits>::FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  cerr << "ERROR:: HierarchicalNF::FindNeighborPairs is not yet implemented. Exiting." << endl;
  exit(1);
}

#endif
