#ifndef PMPL_OPTIMAL_NF_H_
#define PMPL_OPTIMAL_NF_H_

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

    typedef typename MPTraits::CfgType                CfgType;
    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename RoadmapType::VID                 VID;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;

    ///@}
    ///@name Construction
    ///@{

    OptimalNF();

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

  protected:

    ///@name Internal State
    ///@{

    std::string m_nfLabel; ///< Label of the underlying neighborhood finder.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
OptimalNF<MPTraits>::
OptimalNF() : NeighborhoodFinderMethod<MPTraits>() {
  this->SetName("OptimalNF");
  this->m_nfType = Type::OPTIMAL;
}


template <typename MPTraits>
OptimalNF<MPTraits>::
OptimalNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node, false) {
  this->SetName("OptimalNF");
  this->m_nfType = Type::OPTIMAL;
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
template <typename InputIterator, typename OutputIterator>
OutputIterator
OptimalNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  auto nfptr = this->GetNeighborhoodFinder(this->m_nfLabel);

  //save k and radius
  this->m_k = nfptr->GetK();
  this->m_radius = nfptr->GetRadius();

  if(nfptr->GetType() == Type::K) {
    // Calculate k rounding up
    nfptr->GetK() = std::min<size_t>(
        std::ceil(2*2.71828*log(_rmp->get_num_vertices())),
        std::distance(_first, _last));
    if(this->m_debug)
      cout << "Finding closest neighbors with k = " << nfptr->GetK() << endl;
  }
  else if(nfptr->GetType() == Type::RADIUS) {
    throw NotImplementedException(WHERE);
    if(this->m_debug)
      cout << "Finding closest neighbors within radius = " << endl;
  }
  else {
    throw RunTimeException(WHERE, "OptimalNF cannot use anything but radius or "
      " k based NF method (but radius is also not implemented).");
  }

  // Compute neighbors
  nfptr->FindNeighbors(_rmp, _first, _last, _fromFullRoadmap, _cfg, _out);

  // Reset k and radius
  nfptr->GetK() = this->m_k;
  nfptr->GetRadius() = this->m_radius;

  return _out;
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
OptimalNF<MPTraits>::
FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
OptimalNF<MPTraits>::
FindNeighbors(GroupRoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
OptimalNF<MPTraits>::
FindNeighborPairs(GroupRoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*----------------------------------------------------------------------------*/

#endif
