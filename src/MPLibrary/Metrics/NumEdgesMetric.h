#ifndef NUM_EDGES_METRIC_H
#define NUM_EDGES_METRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Get number of edges in roadmap.
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NumEdgesMetric : public MetricMethod<MPTraits> {

  public:

    NumEdgesMetric(){
      this->SetName("NumEdgesMetric");
    }

    NumEdgesMetric(XMLNode& _node)
      : MetricMethod<MPTraits>(_node) {
      this->SetName("NumEdgesMetric");
    }

    virtual ~NumEdgesMetric(){}

    virtual double operator()(){
      return this->GetRoadmap()->GetGraph()->get_num_edges();
    }
};

#endif
