#ifndef NUMEDGESMETRIC_H
#define NUMEDGESMETRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief Get number of edges in roadmap.
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
