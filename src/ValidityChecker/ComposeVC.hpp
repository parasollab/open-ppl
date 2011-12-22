#ifndef _COMPOSE_VC_HPP_
#define _COMPOSE_VC_HPP_

//////////////////////////////////////////////////////////////////////////////////////////
#include "ValidityChecker.hpp"
#include "ValidityCheckerMethod.hpp"
#include "MPUtils.h"
#include "boost/bind.hpp"
#include <vector>
#include <string>
#include <functional>
#include <algorithm>
//////////////////////////////////////////////////////////////////////////////////////////


template<typename CFG> class ValidityChecker;
template<typename CFG> struct ComposeFunctor;


template<typename CFG>
class ComposeValidity : public ValidityCheckerMethod 
{  
public:
  enum en_logical_operator { AND, OR };
  typedef ComposeValidity<CFG> this_type;
  typedef typename std::vector<typename ValidityChecker<CFG>::VCMethodPtr>::iterator InputIterator;
  
  ComposeValidity() { }
  ComposeValidity(en_logical_operator _m_logical_operator,
  std::vector<std::string> _m_vec_vcLabel,
  std::vector<typename ValidityChecker<CFG>::VCMethodPtr> _m_vec_vcMethod,
  Compose<InputIterator, logical_and<bool>, ComposeFunctor<CFG> > _com_and,
  Compose<InputIterator, logical_or<bool>,  ComposeFunctor<CFG> > _com_or);

  ComposeValidity(XMLNodeReader& in_Node, MPProblem* in_pProblem);   
  ~ComposeValidity() { }
  
  virtual bool 
  IsValid(Cfg& _cfg, Environment* env, Stat_Class& Stats, 
	  CDInfo& _cdInfo, bool enablePenetration, std::string *pCallName);

  virtual vector< pair<CfgType,CfgType> > GetHistory();
  virtual void ClearHistory();
  
private:
  en_logical_operator m_logical_operator;
  std::vector<std::string> m_vec_vcLabel;
  std::vector<typename ValidityChecker<CFG>::VCMethodPtr> m_vec_vcMethod;
  Compose<InputIterator, logical_and<bool>, ComposeFunctor<CFG> > com_and;
  Compose<InputIterator, logical_or<bool>,  ComposeFunctor<CFG> > com_or;
};

template<typename CFG>
ComposeValidity<CFG>::
ComposeValidity(en_logical_operator _m_logical_operator,
  std::vector<std::string> _m_vec_vcLabel,
  std::vector<typename ValidityChecker<CFG>::VCMethodPtr> _m_vec_vcMethod,
  Compose<InputIterator, logical_and<bool>, ComposeFunctor<CFG> > _com_and,
  Compose<InputIterator, logical_or<bool>,  ComposeFunctor<CFG> > _com_or) : ValidityCheckerMethod(), m_logical_operator(_m_logical_operator), m_vec_vcLabel(_m_vec_vcLabel), com_and(_com_and), com_or(_com_or) {};


template<typename CFG>
ComposeValidity<CFG>::
ComposeValidity(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  ValidityCheckerMethod(in_Node, in_pProblem) 
{
  in_Node.verifyName("ComposeValidity");
  
  string logical_operator = in_Node.stringXMLParameter("operator",true,"","operator");    
  if (logical_operator == "AND" || logical_operator == "and") {
    m_logical_operator = AND;
  } else if (logical_operator == "OR" || logical_operator == "or") {
    m_logical_operator = OR;
  } else {
    std::cout << "unknown logical operator label is read " << std::endl;
    exit(-1);
  }
  
  XMLNodeReader::childiterator citr;
  for (citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr) {
    if (citr->getName() == "vc_method") {
      std::string method_label = citr->stringXMLParameter("method", true, "", "method");
      m_vec_vcLabel.push_back(method_label);
    }
    else {
      citr->warnUnknownNode();
    }
  }
}


template<typename CFG>
bool
ComposeValidity<CFG>::
IsValid(Cfg& _cfg, Environment* env, Stat_Class& Stats, CDInfo& _cdInfo, 
	bool enablePenetration, std::string *pCallName = NULL) 
{
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  
  if (m_vec_vcMethod.size() != m_vec_vcLabel.size()) {
    for(std::vector<std::string>::iterator it = m_vec_vcLabel.begin(); it != m_vec_vcLabel.end(); ++it) {
      m_vec_vcMethod.push_back(vc->GetVCMethod(*it));
    }
  }
  
  ComposeFunctor<CFG> com_func(vc, _cfg, env, Stats, _cdInfo, enablePenetration, pCallName); 
  
  if (m_logical_operator == AND) {
    return com_and(m_vec_vcMethod.begin(), m_vec_vcMethod.end(), logical_and<bool>(), com_func);
  } else if (m_logical_operator == OR) {
    return com_or(m_vec_vcMethod.begin(), m_vec_vcMethod.end(), logical_or<bool>(), com_func);
  } else { 
    std::cout << "unknown label is read " << std::endl;
    return false;
  }        
}

template<typename CFG>
vector< pair<CfgType,CfgType> >
ComposeValidity<CFG>::GetHistory() {
  vector< pair<CfgType,CfgType> > empty;
  return empty;
}

template<typename CFG>
void
ComposeValidity<CFG>::ClearHistory() { }


template<typename CFG>
class ComposeFunctor 
{	    
public:
  ComposeFunctor(ValidityChecker<CFG>* vc, Cfg& _cfg, Environment* env, Stat_Class& Stats, 
		 CDInfo& _cdInfo, bool enablePenetration, std::string *pCallName) : 
    m_vc(vc), m_cfg(_cfg), m_env(env), m_Stats(Stats), m_cdInfo(_cdInfo), 
    m_enablePenetration(enablePenetration), m_pCallName(pCallName) { }
  
  ~ComposeFunctor() {}
  
  bool operator()(typename ValidityChecker<CFG>::VCMethodPtr vcMethodPtr) {
    return m_vc->IsValid(vcMethodPtr, m_cfg, m_env, m_Stats, 
			 m_cdInfo, m_enablePenetration, m_pCallName);
  }
  
private:
  ValidityChecker<CFG>* m_vc;  
  Cfg& m_cfg; 
  Environment* m_env; 
  Stat_Class& m_Stats; 
  CDInfo& m_cdInfo; 
  bool m_enablePenetration; 
  std::string * m_pCallName;
};


#endif// #ifndef _COMPOSE_VC_HPP_
