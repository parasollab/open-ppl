#ifndef OPTIMALNF_H_
#define OPTIMALNF_H_

#include "NeighborhoodFinderMethod.h"

template<class MPTraits>
class OptimalNF : public NeighborhoodFinderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    OptimalNF(string _dmLabel = "", bool _unconnected = false, size_t _k = 5) :
      NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected) {
        this->SetName("OptimalNF");
        this->m_nfType = OPTIMAL;
      }

    OptimalNF(MPProblemType* _problem, XMLNodeReader& _node) :
      NeighborhoodFinderMethod<MPTraits>(_problem, _node, false) {
        this->SetName("OptimalNF");
        this->m_nfType = OPTIMAL;
        m_nfLabel = _node.stringXMLParameter("nfLabel", true, "", "Neighborhood Finder");
      }

    virtual void PrintOptions(ostream& _os) const {
      NeighborhoodFinderMethod<MPTraits>::PrintOptions(_os);
      _os << "\tnfLabel: " << m_nfLabel << endl;
    }

    virtual typename MPProblemType::DistanceMetricPointer GetDMMethod() const{
      return this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel)->GetDMMethod();
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

  protected:
    string m_nfLabel;
};

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
OptimalNF<MPTraits>::FindNeighbors(RoadmapType* _rmp, InputIterator _first, InputIterator _last,
    const CfgType& _cfg, OutputIterator _out) {

  this->IncrementNumQueries();
  this->StartTotalTime();
  this->StartQueryTime();

  typename MPProblemType::NeighborhoodFinderPointer nfptr =
    this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfLabel);

  //save k and radius
  this->m_k = nfptr->GetK();
  this->m_radius = nfptr->GetRadius();

  if(nfptr->GetNFType() == K) {
    // Calculate k
    nfptr->GetK() = min<size_t>(ceil(2*2.71828*log(_rmp->GetGraph()->get_num_vertices())), _last-_first);  // Rounding up
    if (this->m_debug) cout << "Finding closest neighbors with k = " << nfptr->GetK() << endl;
  }
  else if(nfptr->GetNFType() == RADIUS) {
    if(this->m_debug) cout << "Finding closest neighbors within radius = " << endl;
    // Calculate radius
    cerr << "Error:: OptimalNF cannot use radius based method. Optimal radius not yet implemented. Exiting." << endl;
    exit(1);
  }
  else {
    cerr << "Error:: OptimalNF cannot use anything but a radius or k based method. Exiting." << endl;
    exit(1);
  }

  // Compute neighbors
  nfptr->FindNeighbors(_rmp, _first, _last, _cfg, _out);

  //reset k and radius
  nfptr->GetK() = this->m_k;
  nfptr->GetRadius() = this->m_radius;

  this->EndQueryTime();
  this->EndTotalTime();

  //return neighbors
  return _out;
}

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
OptimalNF<MPTraits>::FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  cerr << "ERROR:: OptimalNF::FindNeighborPairs is not yet implemented. Exiting." << endl;
  exit(1);
}

#endif
