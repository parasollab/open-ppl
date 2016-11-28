#ifndef OPTIMAL_NF_H_
#define OPTIMAL_NF_H_

#include "NeighborhoodFinderMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup NeighborhoodFinders
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class OptimalNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;

    typedef typename MPTraits::MPLibrary              MPLibrary;
    typedef typename MPLibrary::DistanceMetricPointer DistanceMetricPointer;

    OptimalNF(string _dmLabel = "", bool _unconnected = false, size_t _k = 5) :
      NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected) {
        this->SetName("OptimalNF");
        this->m_nfType = OPTIMAL;
      }

    OptimalNF(XMLNode& _node) :
      NeighborhoodFinderMethod<MPTraits>(_node, false) {
        this->SetName("OptimalNF");
        this->m_nfType = OPTIMAL;
        m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
      }

    virtual void Print(ostream& _os) const {
      NeighborhoodFinderMethod<MPTraits>::Print(_os);
      _os << "\tnfLabel: " << m_nfLabel << endl;
    }

    virtual DistanceMetricPointer GetDMMethod() const {
      return this->GetNeighborhoodFinder(m_nfLabel)->GetDMMethod();
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

  protected:
    string m_nfLabel;
};

template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
OptimalNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {

  this->IncrementNumQueries();
  this->StartTotalTime();
  this->StartQueryTime();

  auto nfptr = this->GetNeighborhoodFinder(this->m_nfLabel);

  //save k and radius
  this->m_k = nfptr->GetK();
  this->m_radius = nfptr->GetRadius();

  if(nfptr->GetNFType() == K) {
    // Calculate k
    nfptr->GetK() = min<size_t>(ceil(2*2.71828*log(_rmp->GetGraph()->get_num_vertices())), distance(_first, _last));  // Rounding up
    if(this->m_debug)
      cout << "Finding closest neighbors with k = " << nfptr->GetK() << endl;
  }
  else if(nfptr->GetNFType() == RADIUS) {
    if(this->m_debug)
      cout << "Finding closest neighbors within radius = " << endl;
    throw RunTimeException(WHERE, "OptimalNF cannot use radius based NF method."
        " Optimal radius not yet implemented.");
  }
  else {
    throw RunTimeException(WHERE, "OptimalNF cannot use anything but radius or "
      " k based NF method.");
  }

  // Compute neighbors
  nfptr->FindNeighbors(_rmp, _first, _last, _fromFullRoadmap, _cfg, _out);

  //reset k and radius
  nfptr->GetK() = this->m_k;
  nfptr->GetRadius() = this->m_radius;

  this->EndQueryTime();
  this->EndTotalTime();

  //return neighbors
  return _out;
}

template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
OptimalNF<MPTraits>::FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw RunTimeException(WHERE, "FindNeighborPairs is not yet implemented.");
}

#endif
