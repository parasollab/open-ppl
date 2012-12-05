#ifndef VALIDITYCHECKERMETHOD_H
#define VALIDITYCHECKERMETHOD_H

#include <string>
#include "Utilities/MPUtils.h"
#include "ValidityCheckers/CollisionDetection/CDInfo.h"

template<class MPTraits>
class ValidityCheckerMethod : public MPBaseObject<MPTraits> {
  public:
    ValidityCheckerMethod() : MPBaseObject<MPTraits>(), m_validity(true) {}
    ValidityCheckerMethod(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) : MPBaseObject<MPTraits>(_problem, _node), m_validity(true){} 
    virtual ~ValidityCheckerMethod(){}

    bool IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
        CDInfo& _cdInfo, std::string *_callName) {
      if(m_validity)
        return IsValidImpl(_cfg, _env, _stats, _cdInfo, _callName);
      else
        return !IsValidImpl(_cfg, _env, _stats, _cdInfo, _callName);
    }

    virtual bool IsInsideObstacle(const Cfg& _cfg, Environment* _env, CDInfo& _cdInfo){
      cerr << "error: IsInsideObstacle() not defined." << endl;
      exit(-1);
    }

    bool GetValidity() const { return m_validity; }
    void ToggleValidity() { m_validity = !m_validity; }

  protected:
    virtual bool IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
        CDInfo& _cdInfo, std::string *_callName) = 0; 

    bool m_validity;
};

#endif
