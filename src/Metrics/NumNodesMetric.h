#ifndef NUMNODESMETRIC_H
#define NUMNODESMETRIC_H

#include "MetricsMethod.h"

class NumNodesMetric : public MetricsMethod {
  public:

    NumNodesMetric();
    NumNodesMetric(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~NumNodesMetric();

    virtual void PrintOptions(ostream& _os);

    virtual double operator()();
};

#endif
