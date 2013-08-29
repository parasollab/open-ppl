#ifndef VALIDITYCHECKERMETHOD_H
#define VALIDITYCHECKERMETHOD_H

#include <string>
#include "Utilities/MPUtils.h"
#include "ValidityCheckers/CollisionDetection/CDInfo.h"

template<class MPTraits>
class ValidityCheckerMethod : public MPBaseObject<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;

    ValidityCheckerMethod() : MPBaseObject<MPTraits>(), m_validity(true) {}
    ValidityCheckerMethod(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) : MPBaseObject<MPTraits>(_problem, _node), m_validity(true){}
    virtual ~ValidityCheckerMethod(){}

    virtual void PrintOptions(ostream& _os) const {
      _os << this->GetNameAndLabel() << endl;
    }

    bool IsValid(CfgType& _cfg, Environment* _env, StatClass& _stats,
        CDInfo& _cdInfo, std::string* _callName) {
      if(m_validity)
        return IsValidImpl(_cfg, _env, _stats, _cdInfo, _callName);
      else
        return !IsValidImpl(_cfg, _env, _stats, _cdInfo, _callName);
    }

    virtual bool IsInsideObstacle(const CfgType& _cfg, Environment* _env, CDInfo& _cdInfo){
      cerr << "error: IsInsideObstacle() not defined." << endl;
      exit(-1);
    }

    bool GetValidity() const { return m_validity; }
    void ToggleValidity() { m_validity = !m_validity; }

  protected:
    virtual bool IsValidImpl(CfgType& _cfg, Environment* _env, StatClass& _stats,
        CDInfo& _cdInfo, std::string* _callName) = 0;

    bool m_validity;
};

#endif
