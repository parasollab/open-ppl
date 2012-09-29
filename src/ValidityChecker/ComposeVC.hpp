//////////////////////////////////////////////
//ComposeVC.hpp
//
//This class composes one or more validity 
//checker methods.
/////////////////////////////////////////////

#ifndef COMPOSEVC_H
#define COMPOSEVC_H

#include "ValidityCheckerMethod.hpp"
#include "ComposeFunctor.h"

class ComposeValidity : public ValidityCheckerMethod {  
  public:
    enum LogicalOperator { AND, OR };
    typedef std::vector<ElementSet<ValidityCheckerMethod>::MethodPointer>::iterator InputIterator;

    ComposeValidity();
    ComposeValidity(LogicalOperator _logicalOperator,
        std::vector<std::string> _vcLabel,
        std::vector<ElementSet<ValidityCheckerMethod>::MethodPointer> _vcMethod,
        Compose<InputIterator, logical_and<bool>, ComposeFunctor> _and,
        Compose<InputIterator, logical_or<bool>,  ComposeFunctor> _or);
    ComposeValidity(XMLNodeReader& _node, MPProblem* _problem);   
    virtual ~ComposeValidity() {}

    virtual bool 
      IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
          CDInfo& _cdInfo, std::string* _callName = NULL);

  private:
    LogicalOperator m_logicalOperator;
    std::vector<std::string> m_label;
    std::vector<ElementSet<ValidityCheckerMethod>::MethodPointer> m_vcMethod;
    Compose<InputIterator, logical_and<bool>, ComposeFunctor> m_and;
    Compose<InputIterator, logical_or<bool>,  ComposeFunctor> m_or;
};

#endif
