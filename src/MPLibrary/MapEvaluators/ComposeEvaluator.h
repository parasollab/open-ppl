#ifndef COMPOSE_EVALUATION_H_
#define COMPOSE_EVALUATION_H_

#include "MapEvaluatorMethod.h"
#include "EvaluatorFunctor.h"

////////////////////////////////////////////////////////////////////////////////
/// Combines two or more MapEvaluators to produce a compound condition.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class ComposeEvaluator : public MapEvaluatorMethod<MPTraits> {

  public:

    ///\name Local Types
    ///@{

    enum LogicalOperator {AND, OR}; ///< The supported logical operators.

    ///@}
    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::MPLibrary MPLibrary;

    ///@}
    ///\name Construction
    ///@{

    ComposeEvaluator(LogicalOperator _logicalOperator = AND,
        const vector<string>& _evalLabels = vector<string>());
    ComposeEvaluator(XMLNode& _node);

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;
    virtual bool operator()() override;

    ///@}

  private:

    ///\name Internal State
    ///@{

    LogicalOperator m_logicalOperator;
    vector<string> m_evalLabels;

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
ComposeEvaluator<MPTraits>::
ComposeEvaluator(LogicalOperator _logicalOperator,
    const vector<string>& _evalLabels) : MapEvaluatorMethod<MPTraits>(),
    m_logicalOperator(_logicalOperator), m_evalLabels(_evalLabels) {
  this->SetName("ComposeEvaluator");
}


template <typename MPTraits>
ComposeEvaluator<MPTraits>::
ComposeEvaluator(XMLNode& _node) :
    MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("ComposeEvaluator");

  string logicalOperator = _node.Read("operator", true, "", "operator");

  if(logicalOperator == "AND" || logicalOperator == "and")
    m_logicalOperator = AND;
  else if(logicalOperator == "OR" || logicalOperator == "or")
    m_logicalOperator = OR;
  else
    throw ParseException(_node.Where(), "Operator '" + logicalOperator +
        "' is unknown.");

  for(auto& child : _node)
    if(child.Name() == "Evaluator")
      m_evalLabels.push_back(child.Read("label", true, "", "method"));

  if(m_evalLabels.size() < 2)
    throw ParseException(_node.Where(), "Must specify at least two evaluators.");
}


template <typename MPTraits>
void
ComposeEvaluator<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
  for(auto& l : m_evalLabels)
    _os << "\n\tevaluation method = \'" << l << "\'";
  _os << "\n\t operator = " << m_logicalOperator << endl;
}


template <typename MPTraits>
bool
ComposeEvaluator<MPTraits>::
operator()() {
  typedef typename MPLibrary::MapEvaluatorPointer MapEvaluatorPointer;
  vector<MapEvaluatorPointer> evalMethods;
  for(auto l : m_evalLabels)
    evalMethods.push_back(this->GetMapEvaluator(l));

  if(this->m_debug)
    cout << "ComposeEvaluator:: checking evaluators..." << endl;
  switch(m_logicalOperator) {
    case AND:
      for(auto e : evalMethods) {
        bool passed = e->operator()();
        if(this->m_debug)
          cout << "\t" << e->GetNameAndLabel() << ":"
               << (passed ? "passed" : "failed") << endl;
        if(!passed)
          return false;
      }
      return true;
    case OR:
      for(auto e : evalMethods) {
        bool passed = e->operator()();
        if(this->m_debug)
          cout << "\t" << e->GetNameAndLabel() << ":"
               << (passed ? "passed" : "failed") << endl;
        if(passed)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE, "Unknown operator is stated.");
  }
  return false;
}

#endif
