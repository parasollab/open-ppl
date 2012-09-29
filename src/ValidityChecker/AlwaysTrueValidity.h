//////////////////////////////////
// AlwaysTrueValidity
//
// This code defines the validity 
// of the configuration as 'always 
// true.'
//////////////////////////////////

#ifndef ALWAYSTRUEVALIDITY_H
#define ALWAYSTRUEVALIDITY_H

#include "ValidityCheckerMethod.hpp"

class AlwaysTrueValidity : public ValidityCheckerMethod {
  public:
    AlwaysTrueValidity();
    AlwaysTrueValidity(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~AlwaysTrueValidity();

    virtual bool IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
                             CDInfo& _cdInfo, string *_callName);
};

#endif
