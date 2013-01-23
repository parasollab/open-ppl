//////////////////////////////////
// AlwaysTrueValidity
//
// This code defines the validity 
// of the configuration as 'always 
// true.'
//////////////////////////////////

#ifndef ALWAYSTRUEVALIDITY_H
#define ALWAYSTRUEVALIDITY_H

#include "ValidityCheckerMethod.h"

template<class MPTraits>
class AlwaysTrueValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;

    AlwaysTrueValidity(){
      this->m_name = "AlwaysTrueValidity";
    }

    AlwaysTrueValidity(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) 
      : ValidityCheckerMethod<MPTraits>(_problem, _node){
        _node.verifyName("AlwaysTrueValidity");
        this->m_name = "AlwaysTrueValidity";
      }

    virtual ~AlwaysTrueValidity(){}

    virtual bool IsValidImpl(CfgType& _cfg, Environment* _env, StatClass& _stats, 
        CDInfo& _cdInfo, string *_callName){
      _cfg.SetLabel("Lazy", true);
      return true;
    }
};

#endif
