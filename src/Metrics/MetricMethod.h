#ifndef METRICSMETHOD_H_
#define METRICSMETHOD_H_

#include "Utilities/MPUtils.h"

template<class MPTraits>
class MetricMethod : public MPBaseObject<MPTraits> {
  public:
    MetricMethod() {}
    MetricMethod(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) : MPBaseObject<MPTraits>(_problem, _node) {}
    virtual ~MetricMethod(){}

    virtual void Print(ostream& _os) const {
      _os << this->GetNameAndLabel() << endl;
    }

    virtual double operator()() = 0;
};

#endif
