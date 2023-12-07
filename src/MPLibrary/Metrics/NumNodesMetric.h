#ifndef PMPL_NUM_NODES_METRIC_H
#define PMPL_NUM_NODES_METRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Evaluates the number of nodes in the current roadmap.
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
class NumNodesMetric : virtual public MetricMethod {

  public:

    ///@name Construction
    ///@{

    NumNodesMetric();

    NumNodesMetric(XMLNode& _node);

    virtual ~NumNodesMetric() = default;

    ///@}
    ///@name Metric Interface
    ///@{

    virtual double operator()() override;

    ///@}
};

#endif
