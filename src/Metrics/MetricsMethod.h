#ifndef METRICSMETHOD_H_
#define METRICSMETHOD_H_

#include "Utilities/MPUtils.h"

template<class MPTraits>
class MetricsMethod : public MPBaseObject<MPTraits> {
  public:
    MetricsMethod() {}
    MetricsMethod(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) : MPBaseObject<MPTraits>(_problem, _node) {}
    virtual ~MetricsMethod(){}

    virtual void PrintOptions(ostream& _os){
      _os << this->GetName() << endl; 
    }

    virtual double operator()() = 0;
};

#endif
