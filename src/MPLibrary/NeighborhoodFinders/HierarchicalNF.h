#ifndef PMPL_HIERARCHICAL_NF_H_
#define PMPL_HIERARCHICAL_NF_H_

#include "NeighborhoodFinderMethod.h"

#include <iostream>
#include <string>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// Runs two other NeighborhoodFinders in order. The results from the first will
/// be passed as candidates to the second.
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class HierarchicalNF : public NeighborhoodFinderMethod<MPTraits> {

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
    using typename NeighborhoodFinderMethod<MPTraits>::OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    HierarchicalNF();

    HierarchicalNF(XMLNode& _node);

    virtual ~HierarchicalNF();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinder Interface
    ///@{

    template <typename InputIterator>
    void FindNeighbors(RoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    template <typename InputIterator>
    void FindNeighborPairs(RoadmapType* _r,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    template <typename InputIterator>
    void FindNeighbors(GroupRoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out);

    template <typename InputIterator>
    void FindNeighborPairs(GroupRoadmapType* _r,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_nfLabel;  ///< The first NF to use.
    std::string m_nfLabel2; ///< The second NF.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
HierarchicalNF<MPTraits>::
HierarchicalNF() : NeighborhoodFinderMethod<MPTraits>() {
  this->SetName("HierarchicalNF");
  this->m_nfType = Type::OTHER;
}


template <typename MPTraits>
HierarchicalNF<MPTraits>::
HierarchicalNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node, false) {
  this->SetName("HierarchicalNF");
  this->m_nfType = Type::OTHER;

  m_nfLabel = _node.Read("nfLabel", true, "", "Neighbor Finder Method1");
  m_nfLabel2 = _node.Read("nfLabel2", true, "", "Neighbor Finder Method2");
}


template <typename MPTraits>
HierarchicalNF<MPTraits>::
~HierarchicalNF() = default;

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
HierarchicalNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tnfLabel: " << m_nfLabel
      << "\n\tnfLabel2: " << m_nfLabel2
      << std::endl;
}

/*----------------------- NeighborhoodFinder Interface -----------------------*/

template <typename MPTraits>
template <typename InputIterator>
void
HierarchicalNF<MPTraits>::
FindNeighbors(RoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  auto nf2 = this->GetNeighborhoodFinder(m_nfLabel2);

  std::vector<Neighbor> closest;
  nf->FindNeighbors(_r, _first, _last, _fromFullRoadmap, _cfg,
      std::back_inserter(closest));

  std::vector<VID> closestVID;
  closestVID.reserve(closest.size());
  for(const auto& p : closest)
    closestVID.push_back(p.target);

  nf2->FindNeighbors(_r, closestVID.begin(), closestVID.end(), false,
      _cfg, _out);
}


template <typename MPTraits>
template <typename InputIterator>
void
HierarchicalNF<MPTraits>::
FindNeighborPairs(RoadmapType* _r,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


template <typename MPTraits>
template <typename InputIterator>
void
HierarchicalNF<MPTraits>::
FindNeighbors(GroupRoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


template <typename MPTraits>
template <typename InputIterator>
void
HierarchicalNF<MPTraits>::
FindNeighborPairs(GroupRoadmapType* _r,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*----------------------------------------------------------------------------*/

#endif
