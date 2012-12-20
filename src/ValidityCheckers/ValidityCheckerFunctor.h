#ifndef VALIDITYCHECKERFUNCTOR_H
#define VALIDITYCHECKERFUNCTOR_H

class ValidityCheckerFunctor {
  public:
    ValidityCheckerFunctor(Cfg& _cfg, Environment* _env, StatClass& _stats, 
               CDInfo& _cdInfo, string* _callName) : 
    m_cfg(_cfg), m_env(_env), m_stats(_stats), m_cdInfo(_cdInfo), 
    m_callName(_callName) {}

    ~ValidityCheckerFunctor() {}

    template<class ValidityCheckerPointer>
      bool operator()(ValidityCheckerPointer _vcMethodPtr){
        return _vcMethodPtr->IsValid(m_cfg, m_env, m_stats, m_cdInfo, m_callName);
      }

  private:
    Cfg& m_cfg;
    Environment* m_env;
    StatClass& m_stats;
    CDInfo& m_cdInfo;
    string* m_callName;
};

#endif
