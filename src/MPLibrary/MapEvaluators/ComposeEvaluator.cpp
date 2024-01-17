#include "ComposeEvaluator.h"

#include "MPLibrary/MPLibrary.h"

#include <algorithm>

/*----------------------------- Construction ---------------------------------*/

ComposeEvaluator::
ComposeEvaluator() {
  this->SetName("ComposeEvaluator");
}


ComposeEvaluator::
ComposeEvaluator(XMLNode& _node) : MapEvaluatorMethod(_node) {
  this->SetName("ComposeEvaluator");

  std::string logicalOperator = _node.Read("operator", true, "", "operator");
  std::transform(logicalOperator.begin(), logicalOperator.end(),
                 logicalOperator.begin(), ::tolower);

  if(logicalOperator == "and")
    m_logicalOperator = AND;
  else if(logicalOperator == "or")
    m_logicalOperator = OR;
  else
    throw ParseException(_node.Where()) << "Operator '" << logicalOperator
                                        << "' is unknown.";

  for(auto& child : _node)
    if(child.Name() == "Evaluator")
      m_evalLabels.push_back(child.Read("label", true, "", "method"));

  if(m_evalLabels.size() < 2)
    throw ParseException(_node.Where()) << "Must specify at least two evaluators.";
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void
ComposeEvaluator::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel();
  for(const auto& l : m_evalLabels)
    _os << "\n\tevaluation method: " << l;
  _os << "\n\toperator: " << (m_logicalOperator == AND ? "AND" : "OR")
      << std::endl;
}


void
ComposeEvaluator::
Initialize() {
  for(const auto& label : m_evalLabels)
    this->GetMPLibrary()->GetMapEvaluator(label)->Initialize();
}


bool
ComposeEvaluator::
operator()() {
  if(this->m_debug)
    std::cout << "ComposeEvaluator::" << std::endl;

  switch(m_logicalOperator) {
    case AND:
      for(const auto& label : m_evalLabels) {
        auto e = this->GetMPLibrary()->GetMapEvaluator(label);

        // Print the name first so that we can see which evaluator this is if
        // its debug flag is also on.
        if(this->m_debug)
          std::cout << "\t" << e->GetNameAndLabel() << ": "
                    << std::endl;

        const bool passed = e->operator()();

        if(this->m_debug)
          std::cout << "\t" << e->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;

        if(!passed)
          return false;
      }
      return true;
    case OR:
      for(const auto& label : m_evalLabels) {
        auto e = this->GetMPLibrary()->GetMapEvaluator(label);

        // Print the name first so that we can see which evaluator this is if
        // its debug flag is also on.
        if(this->m_debug)
          std::cout << "\t" << e->GetNameAndLabel() << ": "
                    << std::endl;

        const bool passed = e->operator()();

        if(this->m_debug)
          std::cout << "\t" << e->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;
        if(passed)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE) << "Unknown operator is stated.";
  }

  return false;
}
