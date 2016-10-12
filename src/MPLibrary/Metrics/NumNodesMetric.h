#ifndef NUMNODESMETRIC_H
#define NUMNODESMETRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief Get number of nodes in roadmap.
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
