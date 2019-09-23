#ifndef PMPL_TRUE_EVALUATION_H_
#define PMPL_TRUE_EVALUATION_H_

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// This evaluator always returns true.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TrueEvaluation : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Construction
    ///@{

    TrueEvaluation();

    TrueEvaluation(XMLNode& _node);

    virtual ~TrueEvaluation() = default;

    ///@}
    ///@name MPBaseObject Overrides

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name MapEvaluateorMethod Overrides
    ///@{

    virtual bool operator()() override;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
TrueEvaluation<MPTraits>::
TrueEvaluation() {
  this->SetName("TrueEvaluation");
}


template <typename MPTraits>
TrueEvaluation<MPTraits>::
TrueEvaluation(XMLNode& _node)
    : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("TrueEvaluation");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
TrueEvaluation<MPTraits>::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel() << std::endl;
}

/*---------------------- MapEvaluatorMethod Overrides ------------------------*/

template <typename MPTraits>
bool
TrueEvaluation<MPTraits>::
operator()() {
  return true;
}

/*----------------------------------------------------------------------------*/

#endif
