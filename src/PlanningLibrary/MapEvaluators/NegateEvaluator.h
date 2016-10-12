#ifndef NEGATEEVALUATOR_H_
#define NEGATEEVALUATOR_H_

#include "MapEvaluatorMethod.h"
#include "EvaluatorFunctor.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief TODO.
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NegateEvaluator : public MapEvaluatorMethod<MPTraits> {
  public:

    typedef typename MPTraits::MPProblemType MPProblemType;

    NegateEvaluator(string _label = "");
    NegateEvaluator(XMLNode& _node);
    ~NegateEvaluator(){}

    virtual void Print(ostream& _os) const;

    virtual bool operator()();

  private:
    string m_evalLabel;
};

template <typename MPTraits>
NegateEvaluator<MPTraits>::NegateEvaluator(string _label) : m_evalLabel(_label){
  this->SetName("NegateEvaluator");
}

template <typename MPTraits>
NegateEvaluator<MPTraits>::NegateEvaluator(XMLNode& _node) :
    MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("NegateEvaluator");
  m_evalLabel = _node.Read("evalLabel", true, "", "Evaluator Label");
}

template <typename MPTraits>
void
NegateEvaluator<MPTraits>::Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl << "evaluation method = " << m_evalLabel << endl;
}

template <typename MPTraits>
bool
NegateEvaluator<MPTraits>::operator()() {
  typedef typename MPTraits::PlanningLibraryType::MapEvaluatorPointer MapEvaluatorPointer;
  typedef typename vector<MapEvaluatorPointer>::iterator MEIterator;
  typedef EvaluatorFunctor<MapEvaluatorPointer> EvalFunctor;

  vector<MapEvaluatorPointer> evalMethods;
  evalMethods.push_back(this->GetMapEvaluator(m_evalLabel));

  EvalFunctor comFunc;

  ComposeNegate<MEIterator, EvalFunctor> negate;

  return negate(evalMethods.begin(), comFunc);
}


#endif
