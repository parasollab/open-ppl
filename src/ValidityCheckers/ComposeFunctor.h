#ifndef COMPOSEFUNCTOR_H
#define COMPOSEFUNCTOR_H

#include "MPUtils.h"
#include <string>

class ValidityCheckerMethod;

class ComposeFunctor {	    
  public:
    ComposeFunctor(Cfg& _cfg, Environment* _env, StatClass& _stats, 
        CDInfo& _cdInfo, std::string* _callName);
    ~ComposeFunctor();

    bool operator()(ElementSet<ValidityCheckerMethod>::MethodPointer _vcMethodPtr);

  private:
    Cfg& m_cfg; 
    Environment* m_env; 
    StatClass& m_stats; 
    CDInfo& m_cdInfo; 
    std::string * m_callName;
};

#endif
