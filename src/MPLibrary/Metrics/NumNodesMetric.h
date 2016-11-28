#ifndef NUM_NODES_METRIC_H
#define NUM_NODES_METRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Get number of nodes in roadmap.
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NumNodesMetric : public MetricMethod<MPTraits> {

  public:

    NumNodesMetric(){
      this->SetName("NumNodesMetric");
    }

    NumNodesMetric(XMLNode& _node) : MetricMethod<MPTraits>(_node){
      this->SetName("NumNodesMetric");
    }

    virtual ~NumNodesMetric(){}

    virtual double operator()(){
      return this->GetRoadmap()->GetGraph()->get_num_vertices();
    }
};

#endif
