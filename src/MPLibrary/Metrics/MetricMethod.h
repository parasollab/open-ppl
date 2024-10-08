#ifndef PMPL_METRIC_METHOD_H_
#define PMPL_METRIC_METHOD_H_

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/MPUtils.h"


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref Metrics.
///
/// MetricMethod has one main function, @c operator() which returns a value
/// associated with the roadmap for that metric.
/// @usage
/// @code
/// MetricPointer m = this->GetMetric(m_mLabel);
/// double v = (*m)(); //call as function object
/// double v2 = m->operator()(); //call with pointer notation
/// @endcode
///
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
class MetricMethod : public MPBaseObject {

  public:

    ///@name Construction
    ///@{

    MetricMethod() = default;

    MetricMethod(XMLNode& _node);

    virtual ~MetricMethod() = default;

    ///@}
    ///@name Interface
    ///@{

    /// Compute a metric on the current roadmap.
    /// @return Metric value
    virtual double operator()() = 0;

    ///@}

};

#endif
