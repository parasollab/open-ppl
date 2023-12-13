#include "ComposeValidity.h"

#include "MPLibrary/MPLibrary.h"

#include <algorithm>

/*------------------------------- Construction -------------------------------*/

ComposeValidity::
ComposeValidity() {
  this->SetName("ComposeValidity");
}


ComposeValidity::
ComposeValidity(XMLNode& _node) : ValidityCheckerMethod(_node) {
  this->SetName("ComposeValidity");

  std::string logicalOperator = _node.Read("operator", true, "", "operator");
  std::transform(logicalOperator.begin(), logicalOperator.end(),
                 logicalOperator.begin(), ::tolower);

  if(logicalOperator == "and")
    m_operator = AND;
  else if(logicalOperator == "or")
    m_operator = OR;
  else
    throw ParseException(_node.Where()) << "Operator '" << logicalOperator
                                        << "' is unknown.";

  for(auto& child : _node)
    if(child.Name() == "ValidityChecker")
      m_vcLabels.push_back(child.Read("label", true, "",
          "ValidityChecker method to include in this set."));

  if(m_vcLabels.size() < 2)
    throw ParseException(_node.Where()) << "Must specify at least two methods.";
}

/*---------------------- ValidityCheckerMethod Overrides ---------------------*/

bool
ComposeValidity::
IsValidImpl(Cfg& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  if(this->m_debug)
    std::cout << "ComposeValidity:: checking validity..."
              << std::endl;

  switch(m_operator) {
    case AND:
      for(auto label : m_vcLabels) {
        auto vc = this->GetMPLibrary()->GetValidityChecker(label);
        const bool passed = vc->IsValid(_cfg, _cdInfo, _callName);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;

        if(!passed)
          return false;
      }
      return true;
    case OR:
      for(auto label : m_vcLabels) {
        auto vc = this->GetMPLibrary()->GetValidityChecker(label);
        const bool passed = vc->IsValid(_cfg, _cdInfo, _callName);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;

        if(passed)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE, "Unknown operator is stated.");
  }
  return false;
}

/*----------------------------------------------------------------------------*/
