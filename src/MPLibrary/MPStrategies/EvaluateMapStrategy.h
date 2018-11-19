#ifndef PMPL_EVALUATE_MAP_STRATEGY_H_
#define PMPL_EVALUATE_MAP_STRATEGY_H_

#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "Utilities/XMLNode.h"

#include <iostream>
#include <string>


////////////////////////////////////////////////////////////////////////////////
/// Evaluate the current roadmap using a designated map evaluator. This is
/// mostly useful in simulations where we need to verify/analyze some property
/// of a roadmap or extract a path without modifying the graph.
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class EvaluateMapStrategy : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Construction
    ///@{

    EvaluateMapStrategy();

    EvaluateMapStrategy(XMLNode& _node);

    virtual ~EvaluateMapStrategy() = default;

    ///@}
    ///@name MPStrategy Overrides
    ///@{

    virtual void Run() override;
    virtual void Finalize() override;

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
EvaluateMapStrategy<MPTraits>::
EvaluateMapStrategy() {
  this->SetName("EvaluateMapStrategy");
}


template <typename MPTraits>
EvaluateMapStrategy<MPTraits>::
EvaluateMapStrategy(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("EvaluateMapStrategy");
}

/*----------------------- MPStrategyMethod Overrides -------------------------*/

template <typename MPTraits>
void
EvaluateMapStrategy<MPTraits>::
Run() {
  this->EvaluateMap();
}


template <typename MPTraits>
void
EvaluateMapStrategy<MPTraits>::
Finalize() {
  // Don't print roadmaps or stats for this strategy.
}

/*----------------------------------------------------------------------------*/

#endif
