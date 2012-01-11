#ifndef ALWAYSTRUEVALIDITY_H_
#define ALWAYSTRUEVALIDITY_H_

#include "ValidityCheckerMethod.hpp"

template<typename CFG>
class AlwaysTrueValidity : public ValidityCheckerMethod {
 public:
  AlwaysTrueValidity() : ValidityCheckerMethod() {}
  AlwaysTrueValidity(XMLNodeReader& in_Node, MPProblem* in_pProblem) : ValidityCheckerMethod(in_Node, in_pProblem) {
    in_Node.verifyName("AlwaysTrueValidity");
  }
  virtual ~AlwaysTrueValidity() {}
  
  virtual bool IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
                       CDInfo& _cdInfo, bool _enablePenetration, string *_callName) {
    return true;
  }

  virtual vector<pair<CfgType, CfgType> > GetHistory() {
    return vector<pair<CfgType, CfgType> >();
  }

  virtual void ClearHistory() {}
};

#endif
