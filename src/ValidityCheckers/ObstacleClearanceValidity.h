#ifndef OBSTACLECLEARANCEVALIDITY_H_
#define OBSTACLECLEARANCEVALIDITY_H_

#include "ValidityCheckerMethod.h"

class ObstacleClearanceValidity : public ValidityCheckerMethod {
  public:
    ObstacleClearanceValidity(const ClearanceParams& _cParams = ClearanceParams());
    ObstacleClearanceValidity(XMLNodeReader& _node, MPProblem* _problem);

    virtual ~ObstacleClearanceValidity() { }

    virtual bool IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
        CDInfo& _cdInfo, string* _callName);

  private:
    ClearanceParams m_cParams;
};

#endif
