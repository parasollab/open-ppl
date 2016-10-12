#ifndef TRUEEVALUATION_H
#define TRUEEVALUATION_H

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief TODO.
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TrueEvaluation : public MapEvaluatorMethod<MPTraits> {
  public:

    TrueEvaluation();
    TrueEvaluation(XMLNode& _node);
    virtual ~TrueEvaluation() = default;

    virtual void Print(ostream& _os) const;

    virtual bool operator()() {return true;}
    virtual bool operator()(int _regionID) {return true;}
};

template <typename MPTraits>
TrueEvaluation<MPTraits>::TrueEvaluation() {
  this->SetName("TrueEvaluation");
}

template <typename MPTraits>
TrueEvaluation<MPTraits>::TrueEvaluation(XMLNode& _node)
  : MapEvaluatorMethod<MPTraits>(_node) {
    this->SetName("TrueEvaluation");
}


template <typename MPTraits>
void
TrueEvaluation<MPTraits>::Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
  _os << "True Evaluator always returns true, no options present." << endl;
}

#endif
