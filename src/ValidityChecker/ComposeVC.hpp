//////////////////////////////////////////////
//ComposeVC.hpp
//
//This class composes one or more validity 
//checker methods.
/////////////////////////////////////////////

#ifndef COMPOSEVC_HPP_
#define COMPOSEVC_HPP_

#include "ValidityChecker.hpp"
#include "ValidityCheckerMethod.hpp"
#include "MPUtils.h"
#include "boost/bind.hpp"
#include <vector>
#include <string>
#include <functional>
#include <algorithm>

template<typename CFG> class ValidityChecker;
template<typename CFG> struct ComposeFunctor;

template<typename CFG>
class ComposeValidity : public ValidityCheckerMethod {  
  public:
    enum LogicalOperator { AND, OR };
    typedef typename std::vector<typename ValidityChecker<CFG>::VCMethodPtr>::iterator InputIterator;

    ComposeValidity() { }
    ComposeValidity(LogicalOperator _logicalOperator,
        std::vector<string> _vcLabel,
        std::vector<typename ValidityChecker<CFG>::VCMethodPtr> _vcMethod,
        Compose<InputIterator, logical_and<bool>, ComposeFunctor<CFG> > _and,
        Compose<InputIterator, logical_or<bool>,  ComposeFunctor<CFG> > _or);

    ComposeValidity(XMLNodeReader& _node, MPProblem* _problem);   
    ~ComposeValidity() { }

    virtual bool 
      IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
          CDInfo& _cdInfo, bool _enablePenetration, string* _callName);

  private:
    LogicalOperator m_logicalOperator;
    std::vector<string> m_label;
    std::vector<typename ValidityChecker<CFG>::VCMethodPtr> m_vcMethod;
    Compose<InputIterator, logical_and<bool>, ComposeFunctor<CFG> > m_and;
    Compose<InputIterator, logical_or<bool>,  ComposeFunctor<CFG> > m_or;
};

template<typename CFG>
ComposeValidity<CFG>::
ComposeValidity(LogicalOperator _operator,
    std::vector<string> _vcLabel,
    std::vector<typename ValidityChecker<CFG>::VCMethodPtr> _vcMethod,
    Compose<InputIterator, logical_and<bool>, ComposeFunctor<CFG> > _and,
    Compose<InputIterator, logical_or<bool>,  ComposeFunctor<CFG> > _or) : ValidityCheckerMethod(), m_logicalOperator(_operator), m_label(_vcLabel), m_and(_and), m_or(_or) {}


template<typename CFG>
ComposeValidity<CFG>::
ComposeValidity(XMLNodeReader& _node, MPProblem* _problem) :
  ValidityCheckerMethod(_node, _problem) {
  _node.verifyName("ComposeValidity");

  string logicalOperator = _node.stringXMLParameter("operator",true,"","operator");    
  if (logicalOperator == "AND" || logicalOperator == "and") {
    m_logicalOperator = AND;
  } else if (logicalOperator == "OR" || logicalOperator == "or") {
    m_logicalOperator = OR;
  } else {
    std::cout << "unknown logical operator label is read " << std::endl;
    exit(-1);
  }

  XMLNodeReader::childiterator citr;
  for (citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
    if (citr->getName() == "vc_method") {
      string methodLabel = citr->stringXMLParameter("method", true, "", "method");
      m_label.push_back(methodLabel);
    }
    else {
      citr->warnUnknownNode();
    }
  }
}


template<typename CFG>
bool
ComposeValidity<CFG>::
IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, CDInfo& _cdInfo, 
    bool _enablePenetration, string* _callName = NULL) {
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();

  if (m_vcMethod.size() != m_label.size()) {
    for(std::vector<string>::iterator it = m_label.begin(); it != m_label.end(); ++it) {
      m_vcMethod.push_back(vc->GetVCMethod(*it));
    }
  }

  ComposeFunctor<CFG> comFunc(vc, _cfg, _env, _stats, _cdInfo, _enablePenetration, _callName); 

  if (m_logicalOperator == AND) {
    return m_and(m_vcMethod.begin(), m_vcMethod.end(), logical_and<bool>(), comFunc);
  } else if (m_logicalOperator == OR) {
    return m_or(m_vcMethod.begin(), m_vcMethod.end(), logical_or<bool>(), comFunc);
  } else { 
    std::cout << "read unknown label" << std::endl;
    return false;
  }        
}

template<typename CFG>
class ComposeFunctor {	    
  public:
    ComposeFunctor(ValidityChecker<CFG>* _vc, Cfg& _cfg, Environment* _env, StatClass& _stats, 
        CDInfo& _cdInfo, bool _enablePenetration, string* _callName) : 
      m_vc(_vc), m_cfg(_cfg), m_env(_env), m_stats(_stats), m_cdInfo(_cdInfo), 
      m_enablePenetration(_enablePenetration), m_callName(_callName) { }

    ~ComposeFunctor() {}

    bool operator()(typename ValidityChecker<CFG>::VCMethodPtr vcMethodPtr) {
      return m_vc->IsValid(vcMethodPtr, m_cfg, m_env, m_stats, 
          m_cdInfo, m_enablePenetration, m_callName);
    }

  private:
    ValidityChecker<CFG>* m_vc;  
    Cfg& m_cfg; 
    Environment* m_env; 
    StatClass& m_stats; 
    CDInfo& m_cdInfo; 
    bool m_enablePenetration; 
    string * m_callName;
};


#endif
