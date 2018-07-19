#ifndef PMPL_MPNN_NEIGHBORHOOD_FINDER_H_
#define PMPL_MPNN_NEIGHBORHOOD_FINDER_H_

#include "NeighborhoodFinderMethod.h"
#include "MPNNWrapper.h"

#include <vector>
#include <functional>


////////////////////////////////////////////////////////////////////////////////
/// TODO
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPNNNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename RoadmapType::VID            VID;
    typedef typename RoadmapType::GraphType      GraphType;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType      GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    MPNNNF(std::string _dmLabel = "", bool _unconnected = false, size_t _k = 5);

    MPNNNF(XMLNode& _node);

    virtual ~MPNNNF();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinder Interface
    ///@{

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(RoadmapType* _rmp,
        InputIterator _first, InputIterator _last, const CfgType& _cfg,
        OutputIterator _out);

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

    ///@name Helpers
    ///@{

    void UpdateInternalModel();

    ///@}
    ///@name Internal State
    ///@{

    size_t m_roadmapVersion{0}; ///< Roadmap ver. on last model update.
    double m_epsilon{0};        ///< Approximateness used by internal kd-tree

    std::unique_ptr<MPNNWrapper> m_kdTree; ///< Internal model.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
MPNNNF<MPTraits>::
MPNNNF(std::string _dmLabel, bool _unconnected, size_t _k)
  : NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected) {
  this->SetName("MPNNNF");
  this->m_nfType = K;
  this->m_k = _k;
}


template <typename MPTraits>
MPNNNF<MPTraits>::
MPNNNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("MPNNNF");
  this->m_nfType = K;
  this->m_k = _node.Read("k", true, 5, 0, MAX_INT, "Number of neighbors to find");
}


template <typename MPTraits>
MPNNNF<MPTraits>::
~MPNNNF() = default;

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
MPNNNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tk: " << this->m_k << endl;
}

/*----------------------- NeighborhoodFinder Interface -----------------------*/

template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
MPNNNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp, InputIterator _first, InputIterator _last,
    const CfgType& _cfg, OutputIterator _out) {
  GraphType* g = _rmp->GetGraph();
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  this->UpdateInternalModel();

  std::vector<Neighbor> closest;
  m_kdTree->KClosest(_cfg.GetData(), this->m_k, std::back_inserter(closest));

  // Reverse order
  return std::copy(closest.begin(), closest.end(), _out);
}


template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
MPNNNF<MPTraits>::
FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


template <typename InputIterator, typename OutputIterator>
OutputIterator
MPNNNF<MPTraits>::
FindNeighbors(GroupRoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


template <typename InputIterator, typename OutputIterator>
OutputIterator
MPNNNF<MPTraits>::
FindNeighborPairs(GroupRoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
MPNNNF<MPTraits>::
UpdateInternalModel() {
  MethodTimer mt(this->GetStatClass(), "MPNNNF::UpdateInternalModel");

  GraphType* g = this->GetRoadmap()->GetGraph();

  // Check if the model is already current.
  const size_t version = g->GetTimestamp();
  if(version == m_roadmapVersion)
    return;
  m_roadmapVersion = version;

  // We are still here, so the model will be rebuilt.

  // Create kdtree structure
  auto mb = this->GetTask()->GetRobot()->GetMultiBody();
  std::vector<int> topology(mb->DOF(), 1);
  m_kdTree = std::unique_ptr<MPNNWrapper>(
      new MPNNWrapper(topology, g->Size(), this->GetK(), m_epsilon));

  // Populate kdtree with the roadmap vertices.
  for(auto v = g->begin(); v != g->end(); v++)
    m_kdTree->add_node(v->property(), v->descriptor());
}

/*----------------------------------------------------------------------------*/

#endif
