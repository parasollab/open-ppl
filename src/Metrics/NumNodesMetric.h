#ifndef NUMNODESMETRIC_H
#define NUMNODESMETRIC_H

#include "MetricsMethod.h"

class NumNodesMetric : public MetricsMethod {
  public:

    NumNodesMetric();
    NumNodesMetric(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~NumNodesMetric();

    virtual void PrintOptions(ostream& _os);

    virtual double operator()() {
      return operator()(GetMPProblem()->CreateMPRegion());
    }
    virtual double operator()(int _regionID);
};

#endif
