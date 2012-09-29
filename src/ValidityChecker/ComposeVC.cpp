#include "ComposeVC.hpp"
#include "MPProblem.h"
#include "ValidityChecker.hpp"

ComposeValidity::
ComposeValidity() : ValidityCheckerMethod() {
  m_name = "ComposeValidity";
}

ComposeValidity::
ComposeValidity(LogicalOperator _operator,
    std::vector<string> _vcLabel,
    std::vector<ElementSet<ValidityCheckerMethod>::MethodPointer> _vcMethod,
    Compose<InputIterator, logical_and<bool>, ComposeFunctor> _and,
    Compose<InputIterator, logical_or<bool>,  ComposeFunctor> _or) : 
  ValidityCheckerMethod(), m_logicalOperator(_operator), m_label(_vcLabel), m_and(_and), m_or(_or) {
  m_name = "ComposeValidity";
}

ComposeValidity::
ComposeValidity(XMLNodeReader& _node, MPProblem* _problem) :
    ValidityCheckerMethod(_node, _problem) {
  _node.verifyName("ComposeValidity");
  m_name = "ComposeValidity";

  string logicalOperator = _node.stringXMLParameter("operator",true,"","operator");    
  if (logicalOperator == "AND" || logicalOperator == "and") {
    m_logicalOperator = AND;
  } else if (logicalOperator == "OR" || logicalOperator == "or") {
    m_logicalOperator = OR;
  } else {
    std::cout << "unknown logical operator label is read " << std::endl;
    exit(-1);
  }

  for (XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
    if (citr->getName() == "vc_method") {
      string methodLabel = citr->stringXMLParameter("method", true, "", "method");
      m_label.push_back(methodLabel);
    } else {
      citr->warnUnknownNode();
    }
  }
}

bool
ComposeValidity::
IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, CDInfo& _cdInfo, string* _callName) {
  ValidityChecker* vc = this->GetMPProblem()->GetValidityChecker();

  if (m_vcMethod.size() != m_label.size()) {
    for(std::vector<string>::iterator it = m_label.begin(); it != m_label.end(); ++it) {
      m_vcMethod.push_back(vc->GetMethod(*it));
    }
  }

  ComposeFunctor comFunc(_cfg, _env, _stats, _cdInfo, _callName); 

  if (m_logicalOperator == AND) {
    return m_and(m_vcMethod.begin(), m_vcMethod.end(), logical_and<bool>(), comFunc);
  } else if (m_logicalOperator == OR) {
    return m_or(m_vcMethod.begin(), m_vcMethod.end(), logical_or<bool>(), comFunc);
  } else { 
    std::cout << "read unknown label" << std::endl;
    return false;
  }        
}

