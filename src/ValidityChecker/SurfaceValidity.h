#ifndef SURFACEVALIDITY_H_
#define SURFACEVALIDITY_H_

#include "ValidityCheckerMethod.h"


class SurfaceValidity : public ValidityCheckerMethod {
 public:
  SurfaceValidity();
  SurfaceValidity(string _vcLabel);
  SurfaceValidity(XMLNodeReader& _node, MPProblem* _problem);
  virtual ~SurfaceValidity() {}

  virtual bool 
    IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
	CDInfo& _cdInfo, string *_callName);

  virtual bool IsInsideObstacle(const Cfg& _cfg, Environment* _env, CDInfo& _cdInfo); 

 private:
  string m_vcLabel;
};

#endif
