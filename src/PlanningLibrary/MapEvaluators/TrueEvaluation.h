#ifndef TRUEEVALUATION_H
#define TRUEEVALUATION_H

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class TrueEvaluation : public MapEvaluatorMethod<MPTraits> {
  public:

    TrueEvaluation();
    TrueEvaluation(typename MPTraits::MPProblemType* _problem, XMLNode& _node);
    virtual ~TrueEvaluation();

    virtual void Print(ostream& _os) const;

    virtual bool operator()() {return true;}
    virtual bool operator()(int _regionID) {return true;}
};

template<class MPTraits>
TrueEvaluation<MPTraits>::TrueEvaluation() {
  this->SetName("TrueEvaluation");
}

template<class MPTraits>
TrueEvaluation<MPTraits>::TrueEvaluation(typename MPTraits::MPProblemType* _problem, XMLNode& _node)
  : MapEvaluatorMethod<MPTraits>(_problem, _node) {
    this->SetName("TrueEvaluation");
}

template<class MPTraits>
TrueEvaluation<MPTraits>::~TrueEvaluation() {
}

template<class MPTraits>
void
TrueEvaluation<MPTraits>::Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
  _os << "True Evaluator always returns true, no options present." << endl;
}

#endif
