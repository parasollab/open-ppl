#ifndef STRATEGY_STATE_EVALUATOR_H_
#define STRATEGY_STATE_EVALUATOR_H_

#include "MapEvaluatorMethod.h"

template <typename MPTraits> class DisassemblyMethod;
template <typename MPTraits> class DisassemblyRRTStrategy;


////////////////////////////////////////////////////////////////////////////////
/// @todo This method needs to be removed as it forces MPStrategies to handle
///       the map evaluator's job. It is retained only for refactoring the
///       disassembly code to an extensible state - do not use it for anything
///       else.
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

  auto method = dynamic_cast<DisassemblyMethod<MPTraits>*>(
      this->GetMPStrategy(m_strategyLabel));
  if(method)
    return method->IsSuccessful();

  auto rrt = dynamic_cast<DisassemblyRRTStrategy<MPTraits>*>(
      this->GetMPStrategy(m_strategyLabel));
  if(rrt)
    return method->IsSuccessful();

  throw RunTimeException(WHERE) << "Strategy '" << m_strategyLabel << "' is "
                                << "not a supported strategy {DisassemblyMethod, "
                                << "DisassemblyRRTStrategy}.";
}

/*----------------------------------------------------------------------------*/

#endif
