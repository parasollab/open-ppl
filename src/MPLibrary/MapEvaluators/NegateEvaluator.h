#ifndef PMPL_NEGATE_EVALUATOR_H_
#define PMPL_NEGATE_EVALUATOR_H_

#include "MapEvaluatorMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Negates the result of another map evaluator.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NegateEvaluator : virtual public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Construction
    ///@{

    NegateEvaluator();

    NegateEvaluator(XMLNode& _node);

    virtual ~NegateEvaluator() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name MapEvaluatorMethod Overrides
    ///@{

    virtual bool operator()() override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_meLabel; ///< The evaluator to negate.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
NegateEvaluator<MPTraits>::
NegateEvaluator() {
  this->SetName("NegateEvaluator");
}


template <typename MPTraits>
NegateEvaluator<MPTraits>::
NegateEvaluator(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("NegateEvaluator");

  m_meLabel = _node.Read("evalLabel", true, "", "Evaluator Label");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
NegateEvaluator<MPTraits>::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel()
      << "\nEvaluation method: " << m_meLabel
      << std::endl;
}

/*-------------------------- MapEvaluator Overrides --------------------------*/

template <typename MPTraits>
bool
NegateEvaluator<MPTraits>::
operator()() {
  auto me = this->GetMapEvaluator(m_meLabel);
  return !(*me)();
}

/*----------------------------------------------------------------------------*/

#endif
