#ifndef NUMEDGESMETRIC_H
#define NUMEDGESMETRIC_H

#include "MetricsMethod.h"

class NumEdgesMetric : public MetricsMethod {
  public:

    NumEdgesMetric();
    NumEdgesMetric(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~NumEdgesMetric();

    virtual void PrintOptions(ostream& _os);

    virtual double operator()();
};

#endif
