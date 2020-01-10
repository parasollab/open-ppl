#ifndef PMPL_SCRATCH_STRATEGY_H_
#define PMPL_SCRATCH_STRATEGY_H_

#include "MPStrategyMethod.h"
#include "nonstd/io.h"


////////////////////////////////////////////////////////////////////////////////
/// Scratch space for testing random stuff in PMPL.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ScratchStrategy : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::VertexSet VertexSet;

    ///@}
    ///@name Construction
    ///@{

    ScratchStrategy();

    ScratchStrategy(XMLNode& _node);

    virtual ~ScratchStrategy() = default;

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Iterate() override;

    ///@}

};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
ScratchStrategy<MPTraits>::
ScratchStrategy() : MPStrategyMethod<MPTraits>() {
  this->SetName("ScratchStrategy");
}


template <typename MPTraits>
ScratchStrategy<MPTraits>::
ScratchStrategy(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("ScratchStrategy");
}

/*------------------------------ Interface -----------------------------------*/

template <typename MPTraits>
void
ScratchStrategy<MPTraits>::
Iterate() {
}

/*----------------------------------------------------------------------------*/

#endif
