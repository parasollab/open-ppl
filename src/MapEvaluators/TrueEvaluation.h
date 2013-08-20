#ifndef TRUEEVALUATION_H
#define TRUEEVALUATION_H

#include "MapEvaluatorMethod.h"

template<class MPTraits>
class TrueEvaluation : public MapEvaluatorMethod<MPTraits> {
  public:

    TrueEvaluation();
    TrueEvaluation(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~TrueEvaluation();

    virtual void PrintOptions(ostream& _os) const;

    virtual bool operator()();
    virtual bool operator()(int _regionID);
};

template<class MPTraits>
TrueEvaluation<MPTraits>::TrueEvaluation() {
  this->SetName("TrueEvaluation");
}

template<class MPTraits>
TrueEvaluation<MPTraits>::TrueEvaluation(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node)
  : MapEvaluatorMethod<MPTraits>(_problem, _node) {
    this->SetName("TrueEvaluation");
}

template<class MPTraits>
TrueEvaluation<MPTraits>::~TrueEvaluation() {
}

template<class MPTraits>
void
TrueEvaluation<MPTraits>::PrintOptions(ostream& _os) const {
  _os << "True Evaluator always returns true, no options present." << endl;
}

template<class MPTraits>
bool
TrueEvaluation<MPTraits>::operator()() {
  return true;
}

template<class MPTraits>
bool
TrueEvaluation<MPTraits>::operator()(int _regionID) {
  return true;
}
#endif
