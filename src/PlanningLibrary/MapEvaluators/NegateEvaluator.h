#ifndef NEGATEEVALUATOR_H_
#define NEGATEEVALUATOR_H_

#include "MapEvaluatorMethod.h"
#include "EvaluatorFunctor.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class NegateEvaluator : public MapEvaluatorMethod<MPTraits> {
  public:

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::MapEvaluatorPointer MapEvaluatorPointer;

    NegateEvaluator(string _label = "");
    NegateEvaluator(MPProblemType* _problem, XMLNode& _node);
    ~NegateEvaluator(){}

    virtual void Print(ostream& _os) const;

    virtual bool operator()();

  private:
    string m_evalLabel;
};

template<class MPTraits>
NegateEvaluator<MPTraits>::NegateEvaluator(string _label) : m_evalLabel(_label){
  this->SetName("NegateEvaluator");
}

template<class MPTraits>
NegateEvaluator<MPTraits>::NegateEvaluator(MPProblemType* _problem, XMLNode& _node) :
    MapEvaluatorMethod<MPTraits>(_problem, _node) {
  this->SetName("NegateEvaluator");
  m_evalLabel = _node.Read("evalLabel", true, "", "Evaluator Label");
}

template<class MPTraits>
void
NegateEvaluator<MPTraits>::Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl << "evaluation method = " << m_evalLabel << endl;
}

template<class MPTraits>
bool
NegateEvaluator<MPTraits>::operator()() {
  typedef typename vector<MapEvaluatorPointer>::iterator MEIterator;
  typedef EvaluatorFunctor<MapEvaluatorPointer> EvalFunctor;

  vector<MapEvaluatorPointer> evalMethods;
  evalMethods.push_back(this->GetMPProblem()->GetMapEvaluator(m_evalLabel));

  EvalFunctor comFunc;

  ComposeNegate<MEIterator, EvalFunctor> negate;

  return negate(evalMethods.begin(), comFunc);
}


#endif
