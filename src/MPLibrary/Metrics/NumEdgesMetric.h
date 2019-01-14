#ifndef PMPL_NUM_EDGES_METRIC_H_
#define PMPL_NUM_EDGES_METRIC_H_

#include "MetricMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Get number of edges in roadmap.
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NumEdgesMetric : public MetricMethod<MPTraits> {

  public:

    NumEdgesMetric() {
      this->SetName("NumEdgesMetric");
    }

    NumEdgesMetric(XMLNode& _node) : MetricMethod<MPTraits>(_node) {
      this->SetName("NumEdgesMetric");
    }

    virtual ~NumEdgesMetric() = default;

    virtual double operator()(){
      if(this->GetGroupTask())
        return this->GetGroupRoadmap()->get_num_edges();
      else
        return this->GetRoadmap()->get_num_edges();
    }
};

#endif
