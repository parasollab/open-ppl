#ifndef OPTIMAL_NF_H_
#define OPTIMAL_NF_H_

#include "NeighborhoodFinderMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// This method appears to be a wrapper for k-nearest neighbors where k is
/// dynamically adjusted based on the size of the roadmap. It is NOT "optimal" in
/// any sense.
/// @TODO Find the appropriate reference for this work and validate that it is
///       worth maintaining... it does not appear that we would ever want to use
///       this.
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class OptimalNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    typedef typename MPTraits::MPLibrary              MPLibrary;
    typedef typename MPLibrary::DistanceMetricPointer DistanceMetricPointer;

    ///@}
    ///@name Construction
    ///@{

    OptimalNF(const std::string& _dmLabel = "", const bool _unconnected = false,
        const size_t _k = 5);

    OptimalNF(XMLNode& _node);

    virtual ~OptimalNF();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

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


    /// Group overloads:
    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(GroupRoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out) {
      throw RunTimeException(WHERE, "Not Supported for groups!");
    }

    template<typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(GroupRoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out) {
      throw RunTimeException(WHERE, "Not Supported for groups!");
    }

    ///@}

  protected:

    ///@name Internal State
    ///@{

    std::string m_nfLabel; ///< Label of the underlying neighborhood finder.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
OptimalNF<MPTraits>::
OptimalNF(const std::string& _dmLabel, const bool _unconnected, const size_t _k)
  : NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected) {
  this->SetName("OptimalNF");
  this->m_nfType = OPTIMAL;
}


template <typename MPTraits>
OptimalNF<MPTraits>::
OptimalNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node, false) {
  this->SetName("OptimalNF");
  this->m_nfType = OPTIMAL;
  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
}


template <typename MPTraits>
OptimalNF<MPTraits>::
~OptimalNF() = default;

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
OptimalNF<MPTraits>::
Initialize() {
  // Set the DM label to be the same as for the underlying NF.
  this->m_dmLabel = this->GetNeighborhoodFinder(m_nfLabel)->GetDMLabel();
}


template <typename MPTraits>
void
OptimalNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tnfLabel: " << m_nfLabel
      << std::endl;
}

/*-------------------- NeighborhoodFinderMethod Interface --------------------*/

template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
OptimalNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(), "OptimalNF::FindNeighbors");
  this->IncrementNumQueries();

  auto nfptr = this->GetNeighborhoodFinder(this->m_nfLabel);

  //save k and radius
  this->m_k = nfptr->GetK();
  this->m_radius = nfptr->GetRadius();

  if(nfptr->GetNFType() == K) {
    // Calculate k rounding up
    nfptr->GetK() = std::min<size_t>(
        ceil(2*2.71828*log(_rmp->GetGraph()->get_num_vertices())),
        distance(_first, _last));
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

  // Reset k and radius
  nfptr->GetK() = this->m_k;
  nfptr->GetRadius() = this->m_radius;

  return _out;
}


template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
OptimalNF<MPTraits>::
FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw RunTimeException(WHERE, "FindNeighborPairs is not yet implemented.");
}

/*----------------------------------------------------------------------------*/

#endif
