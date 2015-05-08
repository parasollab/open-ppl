#ifndef NUMNODESMETRIC_H
#define NUMNODESMETRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief Get number of nodes in roadmap.
/// @tparam MPTraits Motion planning universe
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class NumNodesMetric : public MetricMethod<MPTraits> {
  public:
    NumNodesMetric(){
      this->SetName("NumNodesMetric");
    }

    NumNodesMetric(typename MPTraits::MPProblemType* _problem, XMLNode& _node) : MetricMethod<MPTraits>(_problem, _node){
      this->SetName("NumNodesMetric");
    }

    virtual ~NumNodesMetric(){}

    virtual double operator()(){
      return this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_vertices();
    }
};

#endif
