#ifndef COMPOSEEVALUATION_H_
#define COMPOSEEVALUATION_H_

#include "MapEvaluatorMethod.h"
#include "EvaluatorFunctor.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class ComposeEvaluator : public MapEvaluatorMethod<MPTraits> {
  public:

    enum LogicalOperator { AND, OR };
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::MapEvaluatorPointer MapEvaluatorPointer;

    ComposeEvaluator(LogicalOperator _logicalOperator = AND,
        const vector<string>& _evalLabels = vector<string>());

    ComposeEvaluator(MPProblemType* _problem, XMLNode& _node);
    ~ComposeEvaluator() { }

    virtual void Print(ostream& _os) const;

    virtual bool operator()();

  private:
    LogicalOperator m_logicalOperator;
    vector<string> m_evalLabels;
};

template<class MPTraits>
ComposeEvaluator<MPTraits>::
ComposeEvaluator(LogicalOperator _logicalOperator,
    const vector<string>& _evalLabels) : MapEvaluatorMethod<MPTraits>(),
  m_logicalOperator(_logicalOperator), m_evalLabels(_evalLabels) {
    this->SetName("ComposeEvaluator");
  }

template<class MPTraits>
ComposeEvaluator<MPTraits>::
ComposeEvaluator(MPProblemType* _problem, XMLNode& _node) :
  MapEvaluatorMethod<MPTraits>(_problem, _node) {
    this->SetName("ComposeEvaluator");

    string logicalOperator = _node.Read("operator", true, "", "operator");

    if (logicalOperator == "AND" || logicalOperator == "and")
      m_logicalOperator = AND;
    else if (logicalOperator == "OR" || logicalOperator == "or")
      m_logicalOperator = OR;
    else
      throw ParseException(_node.Where(),
          "Operator '" + logicalOperator + "' is unknown.");

    for(auto& child : _node)
      if(child.Name() == "Evaluator")
        m_evalLabels.push_back(child.Read("label", true, "", "method"));

    if(m_evalLabels.size() < 2)
      throw ParseException(_node.Where(),
          "Must specify at least two evaluators.");
  }

template<class MPTraits>
void
ComposeEvaluator<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl ;
  for(auto& l : m_evalLabels)
    _os << "\n\t evaluation method = \'" << l << "\'";
  _os << "\n\t operator = " << m_logicalOperator << endl;
}

template<class MPTraits>
bool
ComposeEvaluator<MPTraits>::
operator()() {

  vector<MapEvaluatorPointer> evalMethods;
  typedef typename vector<MapEvaluatorPointer>::iterator MEIterator;
  for(vector<string>::iterator it = m_evalLabels.begin(); it != m_evalLabels.end(); ++it) {
    evalMethods.push_back(this->GetMPProblem()->GetMapEvaluator(*it));
  }

  typedef EvaluatorFunctor<MapEvaluatorPointer> EvalFunctor;
  EvalFunctor comFunc;

  if (m_logicalOperator == AND) {
    Compose<MEIterator, logical_and<bool>, EvalFunctor> comAnd;
    return comAnd(evalMethods.begin(), evalMethods.end(), logical_and<bool>(), comFunc);
  }
  else if (m_logicalOperator == OR) {
    Compose<MEIterator, logical_or<bool>, EvalFunctor> comOr;
    return comOr(evalMethods.begin(), evalMethods.end(), logical_or<bool>(), comFunc);
  }
  else {
    cerr << "Warning:: Compose Evaluator unknown operator is stated." << endl;
    return false;
  }
}

#endif
