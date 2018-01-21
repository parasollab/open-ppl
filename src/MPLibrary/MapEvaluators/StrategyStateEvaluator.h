#ifndef STRATEGY_STATE_EVALUATOR_H_
#define STRATEGY_STATE_EVALUATOR_H_

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// A distance evaluator between all bodies of the first robot.
/// It returns true, if all distances are higher than the specified distance.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class StrategyStateEvaluator : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Construction
    ///@{

  StrategyStateEvaluator();

  StrategyStateEvaluator(XMLNode& _node);

    virtual ~StrategyStateEvaluator() = default;

    ///@}
    ///@name MapEvaluatorMethod Interface
    ///@{

    virtual bool operator()() override;

    ///@}

    std::string m_strategyLabel;  ///< Strategy label for evaluation.
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
StrategyStateEvaluator<MPTraits>::
StrategyStateEvaluator() : MapEvaluatorMethod<MPTraits>() {
  this->SetName("StrategyStateEvaluator");
}

template <typename MPTraits>
StrategyStateEvaluator<MPTraits>::
StrategyStateEvaluator(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("StrategyStateEvaluator");
}

/*---------------------- MapEvaluatorMethod Interface ------------------------*/

template <typename MPTraits>
bool
StrategyStateEvaluator<MPTraits>::
operator()() {
  if(m_strategyLabel.empty())
    throw RunTimeException(WHERE, "Must set the strategy label before using "
        "this ME! Make sure the xml label stateMELabel is set for this "
        "strategy");

  return this->GetMPStrategy(m_strategyLabel)->IsSuccessful();
}

/*----------------------------------------------------------------------------*/

#endif
