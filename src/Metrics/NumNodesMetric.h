#ifndef NUMNODESMETRIC_H
#define NUMNODESMETRIC_H

#include "MetricsMethod.h"

template<class MPTraits>
class NumNodesMetric : public MetricsMethod<MPTraits> {
  public:
    NumNodesMetric(){
      this->SetName("NumNodesMetric");
    }

    NumNodesMetric(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) : MetricsMethod<MPTraits>(_problem, _node){
      this->SetName("NumNodesMetric");
    }

    virtual ~NumNodesMetric(){}

    virtual double operator()(){
      return this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_vertices();
    }
};

#endif
