//////////////////////////////////
// AlwaysTrueValidity
//
// This code defines the validity 
// of the configuration as 'always 
// true.'
//////////////////////////////////

#ifndef ALWAYSTRUEVALIDITY_H_
#define ALWAYSTRUEVALIDITY_H_

#include "ValidityCheckerMethod.hpp"

template<typename CFG>
class AlwaysTrueValidity : public ValidityCheckerMethod {
  public:
    AlwaysTrueValidity() : ValidityCheckerMethod() {}

    AlwaysTrueValidity(XMLNodeReader& _node, MPProblem* _problem) :
      ValidityCheckerMethod(_node, _problem) {
        _node.verifyName("AlwaysTrueValidity");
      }

    virtual ~AlwaysTrueValidity() {}

    virtual bool 
      IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
          CDInfo& _cdInfo, bool _enablePenetration, string *_callName) {
        return true;
      }

    virtual vector<pair<CfgType, CfgType> > 
      GetHistory(){
        return vector<pair<CfgType, CfgType> >();
      }

    virtual void 
      ClearHistory(){}
};

#endif
