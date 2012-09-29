#include "ComposeFunctor.h"
#include "ValidityChecker.hpp"

ComposeFunctor::
ComposeFunctor(Cfg& _cfg, Environment* _env, StatClass& _stats, 
               CDInfo& _cdInfo, std::string* _callName) : 
    m_cfg(_cfg), m_env(_env), m_stats(_stats), m_cdInfo(_cdInfo), 
    m_callName(_callName) {
}

ComposeFunctor::
~ComposeFunctor() {}

bool 
ComposeFunctor::
operator()(ElementSet<ValidityCheckerMethod>::MethodPointer _vcMethodPtr) {
  return _vcMethodPtr->IsValid(m_cfg, m_env, m_stats, m_cdInfo, m_callName);
}

