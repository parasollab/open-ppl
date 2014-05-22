#ifndef COMPOSEEVALUATION_H_
#define COMPOSEEVALUATION_H_

#include "MapEvaluatorMethod.h"
#include "EvaluatorFunctor.h"

template <class MPTraits>
class ComposeEvaluator : public MapEvaluatorMethod<MPTraits> {
  public:

    enum LogicalOperator { AND, OR };
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::MapEvaluatorPointer MapEvaluatorPointer;

    ComposeEvaluator(LogicalOperator _logicalOperator = AND,
        const vector<string>& _evalLabels = vector<string>());

    ComposeEvaluator(MPProblemType* _problem, XMLNodeReader& _node);
    ~ComposeEvaluator() { }

    virtual void Print(ostream& _os) const;

    virtual bool operator()();

  private:
    LogicalOperator m_logicalOperator;
    vector<string> m_evalLabels;
};

template<class MPTraits>
ComposeEvaluator<MPTraits>::ComposeEvaluator(LogicalOperator _logicalOperator, const vector<string>& _evalLabels)
  : MapEvaluatorMethod<MPTraits>(), m_logicalOperator(_logicalOperator), m_evalLabels(_evalLabels) {
    this->SetName("ComposeEvaluator");
  }

template<class MPTraits>
ComposeEvaluator<MPTraits>::ComposeEvaluator(MPProblemType* _problem, XMLNodeReader& _node)
  : MapEvaluatorMethod<MPTraits>(_problem, _node) {
    this->SetName("ComposeEvaluator");

    string logicalOperator = _node.stringXMLParameter("operator",true,"","operator");
    if (logicalOperator == "AND" || logicalOperator == "and") {
      m_logicalOperator = AND;
    }
    else if (logicalOperator == "OR" || logicalOperator == "or") {
      m_logicalOperator = OR;
    }
    else {
      cerr << "unknown logical operator label is read " << endl;
      exit(-1);
    }

    XMLNodeReader::childiterator citr;
    for (citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
      if (citr->getName() == "Evaluator") {
        string methodLabel = citr->stringXMLParameter("label", true, "", "method");
        m_evalLabels.push_back(methodLabel);
      }
      else {
        citr->warnUnknownNode();
      }
    }

    if(m_evalLabels.size() < 2){
      cerr << "Error:: ComposeEvaluator should specify at least 2 evaluator labels in the XML" << endl;
      exit(1);
    }
  }

template<class MPTraits>
void
ComposeEvaluator<MPTraits>::Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl ;
  for(vector<string>::const_iterator it = m_evalLabels.begin(); it != m_evalLabels.end(); it++)
    _os << "\n\t evaluation method = \'" << *it << "\'";
  _os << "\n\t operator = " << m_logicalOperator << endl;
}

template<class MPTraits>
bool
ComposeEvaluator<MPTraits>::operator()() {

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
