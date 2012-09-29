#ifndef NEGATEVALIDITY_H
#define NEGATEVALIDITY_H

#include "ValidityCheckerMethod.hpp"
#include "ComposeFunctor.h"

class NegateValidity : public ValidityCheckerMethod {
 public:
  typedef vector<ElementSet<ValidityCheckerMethod>::MethodPointer>::iterator InputIterator;
  
  NegateValidity();
  NegateValidity(string _label, vector<ElementSet<ValidityCheckerMethod>::MethodPointer> _vec, ComposeNegate<InputIterator, ComposeFunctor> _m_composeNegate);
  NegateValidity(XMLNodeReader& _node, MPProblem* _problem);   
  virtual ~NegateValidity() { }
  
  virtual bool 
  IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
	  CDInfo& _cdInfo, std::string *_callName);

private:
  string m_vcLabel;
  vector<ElementSet<ValidityCheckerMethod>::MethodPointer> m_vcMethods;
  ComposeNegate<InputIterator, ComposeFunctor> m_composeNegate;
};

#endif// #ifndef NEGATEVALIDITY_H
