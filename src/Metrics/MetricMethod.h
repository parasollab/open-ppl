#ifndef METRICSMETHOD_H_
#define METRICSMETHOD_H_

#include "Utilities/MPUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief Base algorithm abstraction for \ref Metrics.
///
/// MetricMethod has one main function, @c operator() which returns a value
/// associated with the roadmap for that metric.
///
/// @usage
/// @code
/// MetricPointer m = this->GetMPProblem()->GetMetric(m_mLabel);
/// double v = (*m)(); //call as function object
/// double v2 = m->operator()(); //call with pointer notation
/// @endcode
////////////////////////////////////////////////////////////////////////////////
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
