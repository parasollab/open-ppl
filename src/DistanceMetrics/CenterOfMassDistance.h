#ifndef CENTEROFMASSDISTANCE_H_
#define CENTEROFMASSDISTANCE_H_

#include "DistanceMetricMethod.h"

class CenterOfMassDistance : public DistanceMetricMethod {
  public:
    CenterOfMassDistance();
    CenterOfMassDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~CenterOfMassDistance();

    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
   
};

#endif
