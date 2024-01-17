#ifndef PMPL_NUM_EDGES_METRIC_H_
#define PMPL_NUM_EDGES_METRIC_H_

#include "MetricMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Count the number of edges in roadmap.
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
class NumEdgesMetric : virtual public MetricMethod {

  public:

    ///@name Construction
    ///@{

    NumEdgesMetric();

    NumEdgesMetric(XMLNode& _node);

    virtual ~NumEdgesMetric() = default;

    ///@}
    ///@name MetricMethod Interface
    ///@{

    virtual double operator()() override;

    ///@}

};

#endif
