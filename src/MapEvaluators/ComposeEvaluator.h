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

    ///\name Local Types
    ///@{

    enum LogicalOperator {AND, OR}; ///< The supported logical operators.

    ///@}
    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::MPProblemType            MPProblemType;
    typedef typename MPProblemType::MapEvaluatorPointer MapEvaluatorPointer;

    ///@}
    ///\name Construction
    ///@{

    ComposeEvaluator(LogicalOperator _logicalOperator = AND,
        const vector<string>& _evalLabels = vector<string>());
    ComposeEvaluator(MPProblemType* _problem, XMLNode& _node);

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


template<class MPTraits>
void
ComposeEvaluator<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
  for(auto& l : m_evalLabels)
    _os << "\n\tevaluation method = \'" << l << "\'";
  _os << "\n\t operator = " << m_logicalOperator << endl;
}


template<class MPTraits>
bool
ComposeEvaluator<MPTraits>::
operator()() {
  vector<MapEvaluatorPointer> evalMethods;
  for(auto l : m_evalLabels)
    evalMethods.push_back(this->GetMPProblem()->GetMapEvaluator(l));

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
