#include "DistanceMetricMethod.h"
#include "Cfg.h"
#include "Environment.h"
#include "MetricUtils.h"
#include "MPProblem.h"
#include "CollisionDetection.h"
#include "LocalPlanners.h"

DistanceMetricMethod::
DistanceMetricMethod() {}


DistanceMetricMethod::
DistanceMetricMethod(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : MPBaseObject(_node, _problem) {
  if(_warn)
    _node.warnUnrequestedAttributes();
}


DistanceMetricMethod::~DistanceMetricMethod() {
}

bool DistanceMetricMethod::operator==(const DistanceMetricMethod& _dm) const {
  return GetName() == _dm.GetName();
}

void DistanceMetricMethod::ScaleCfg(Environment* _env, double _length, Cfg& _o, Cfg& _c) {
  _length = fabs(_length); //a distance must be positive
  Cfg* origin = &_o;
  Cfg* outsideCfg = _c.CreateNewCfg();
  // first find an outsite configuration with sufficient size
  while(Distance(_env, *origin, *outsideCfg) < 2*_length)
    for(int i=0; i<outsideCfg->DOF(); ++i)
      outsideCfg->SetSingleParam(i, 2*outsideCfg->GetSingleParam(i));
  // now, using binary search  find a configuration with the approximate length
  Cfg* aboveCfg = outsideCfg->CreateNewCfg();
  Cfg* belowCfg = origin->CreateNewCfg();
  Cfg* currentCfg = _c.CreateNewCfg();
  while (1) {
    for(int i=0; i<currentCfg->DOF(); ++i)
      currentCfg->SetSingleParam(i, (aboveCfg->GetSingleParam(i) + belowCfg->GetSingleParam(i)) / 2.0);
    double magnitude = Distance(_env, *origin, *currentCfg);
    if( (magnitude >= _length*0.9) && (magnitude <= _length*1.1)) 
      break;
    if(magnitude>_length) 
      *aboveCfg = *currentCfg;
    else 
      *belowCfg = *currentCfg; 
  }
  for(int i=0; i<_c.DOF(); ++i)
    _c.SetSingleParam(i, currentCfg->GetSingleParam(i));
  //delete origin;
  delete outsideCfg;
  delete aboveCfg;
  delete belowCfg;
  delete currentCfg;
}

