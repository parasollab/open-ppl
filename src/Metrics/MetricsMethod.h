#ifndef METRICSMETHOD_H_
#define METRICSMETHOD_H_

#include "MPUtils.h"
#include "MPProblem.h"

class MetricsMethod : public MPBaseObject {
  public:

    MetricsMethod() {}
    MetricsMethod(XMLNodeReader& _node, MPProblem* _problem)
      : MPBaseObject(_node, _problem) {}
    virtual ~MetricsMethod() {}

    virtual void PrintOptions(ostream& _os) = 0;

    virtual double operator()() {
      return operator()(GetMPProblem()->CreateMPRegion());
    }
    virtual double operator()(int _regionID) = 0;
};

#endif
