#ifndef PMPL_HIERARCHICAL_NF_H_
#define PMPL_HIERARCHICAL_NF_H_

#include "NeighborhoodFinderMethod.h"

#include <iostream>
#include <string>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// A meta-method which runs multiple NeighborhoodFinders in order. The results
/// from each method are passed as candidates to the next.
///
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
    typedef typename RoadmapType::VertexSet      VertexSet;
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

    virtual ~HierarchicalNF() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinder Overrides
    ///@{

    virtual void SetDMLabel(const std::string& _label) noexcept override;

    virtual const std::string& GetDMLabel() const noexcept override;

    virtual void FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out) override;

    virtual void FindNeighbors(GroupRoadmapType* const _r,
        const GroupCfgType& _cfg, const VertexSet& _candidates,
        OutputIterator _out) override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Templated implementation for both individual and group versions.
    template <typename AbstractRoadmapType>
    void FindNeighborsImpl(AbstractRoadmapType* const _r,
        const typename AbstractRoadmapType::CfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out);

    ///@}
    ///@name Internal State
    ///@{

    std::vector<std::string> m_nfLabels;  ///< The NF labels to use, in order.
    std::vector<Neighbor> m_neighborBuffer; ///< Buffer between method calls.
    VertexSet m_candidateBuffer;            ///< Buffer between method calls.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
HierarchicalNF<MPTraits>::
HierarchicalNF() : NeighborhoodFinderMethod<MPTraits>(Type::OTHER) {
  this->SetName("HierarchicalNF");
}


template <typename MPTraits>
HierarchicalNF<MPTraits>::
HierarchicalNF(XMLNode& _node) :
    NeighborhoodFinderMethod<MPTraits>(_node, Type::OTHER, false) {
  this->SetName("HierarchicalNF");

  for(auto& child : _node)
    if(child.Name() == "NeighborhoodFinder")
      m_nfLabels.push_back(child.Read("label", true, "",
          "NeighborhoodFinder Method"));
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
HierarchicalNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  for(const std::string& label : m_nfLabels)
    _os << "\tLabel: " << label << std::endl;
}

/*-------------------- NeighborhoodFinderMethod Overrides --------------------*/

template <typename MPTraits>
inline
void
HierarchicalNF<MPTraits>::
SetDMLabel(const std::string& _label) noexcept {
  this->GetNeighborhoodFinder(m_nfLabels.back())->SetDMLabel(_label);
}


template <typename MPTraits>
inline
const std::string&
HierarchicalNF<MPTraits>::
GetDMLabel() const noexcept {
  return this->GetNeighborhoodFinder(m_nfLabels.back())->GetDMLabel();
}


template <typename MPTraits>
void
HierarchicalNF<MPTraits>::
FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  this->FindNeighborsImpl(_r, _cfg, _candidates, _out);
}


template <typename MPTraits>
void
HierarchicalNF<MPTraits>::
FindNeighbors(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  this->FindNeighborsImpl(_r, _cfg, _candidates, _out);
}

/*--------------------------------- Helpers ----------------------------------*/


template <typename MPTraits>
template <typename AbstractRoadmapType>
void
HierarchicalNF<MPTraits>::
FindNeighborsImpl(AbstractRoadmapType* const _r,
    const typename AbstractRoadmapType::CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  // Run each method in succession.
  m_candidateBuffer = _candidates;
  for(auto iter = m_nfLabels.begin(); ; ) {
    // Find neighbors with the current method and candidate set.
    m_neighborBuffer.clear();
    this->GetNeighborhoodFinder(*iter)->FindNeighbors(_r, _cfg,
        m_candidateBuffer, std::back_inserter(m_neighborBuffer));

    // Check if this was the last method.
    ++iter;
    const bool lastMethod = iter == m_nfLabels.end();
    if(lastMethod)
      break;

    // This isn't the last method, the discovered neighbors become the
    // candidates.
    m_candidateBuffer.clear();
    for(const auto& neighbor : m_neighborBuffer)
      m_candidateBuffer.insert(neighbor.target);
  }

  // Copy the final result set to the output iterator.
  for(const auto& neighbor : m_neighborBuffer)
    _out = neighbor;
}

/*----------------------------------------------------------------------------*/

#endif
