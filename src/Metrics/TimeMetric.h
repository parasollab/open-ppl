#ifndef TIMEMETRIC_H
#define TIMEMETRIC_H

#include "MetricsMethod.h"
#include "Utilities/MetricUtils.h"

class TimeMetric : public MetricsMethod {
  public:

    TimeMetric();
    TimeMetric(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~TimeMetric();

    virtual void PrintOptions(ostream& _os);

    double operator()();
   
};

#endif
