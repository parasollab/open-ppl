#ifndef METRIC_METHOD_H_
#define METRIC_METHOD_H_

#include "Utilities/MPUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief Base algorithm abstraction for \ref Metrics.
/// @tparam MPTraits Motion planning universe
///
/// MetricMethod has one main function, @c operator() which returns a value
/// associated with the roadmap for that metric.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class MetricMethod : public MPBaseObject<MPTraits> {
  public:

    MetricMethod() {}
    MetricMethod(typename MPTraits::MPProblemType* _problem, XMLNode& _node)
      : MPBaseObject<MPTraits>(_problem, _node) {}
    virtual ~MetricMethod() {}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute a metric from a roadmap
    /// @return Metric value
    ///
    /// @usage
    /// @code
    /// MetricPointer m = this->GetMPProblem()->GetMetric(m_mLabel);
    /// double v = (*m)(); //call as function object
    /// double v2 = m->operator()(); //call with pointer notation
    /// @endcode
    ////////////////////////////////////////////////////////////////////////////
    virtual double operator()() = 0;
};

#endif
