#ifndef VALIDITYCHECKERMETHOD_H
#define VALIDITYCHECKERMETHOD_H

#include <string>
#include "MPUtils.h"


class ValidityCheckerMethod : public MPBaseObject {
 public:
  ValidityCheckerMethod();
  ValidityCheckerMethod(XMLNodeReader& _node, MPProblem* _problem);
  virtual ~ValidityCheckerMethod();
  
  bool IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
	       CDInfo& _cdInfo, std::string *_callName) {
    if(m_validity)
      return IsValidImpl(_cfg, _env, _stats, _cdInfo, _callName);
    else
      return !IsValidImpl(_cfg, _env, _stats, _cdInfo, _callName);
  }
 protected:
  virtual bool IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
                           CDInfo& _cdInfo, std::string *_callName) = 0; 

 public:
  virtual bool isInsideObstacle(const Cfg& _cfg, Environment* _env, CDInfo& _cdInfo);

  bool GetValidity() const { return m_validity; }
  void ToggleValidity() { m_validity = !m_validity; }
 
  bool m_validity;
};
#endif // End #ifndef VALIDITYCHECKERMETHOD_H
