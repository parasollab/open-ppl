#ifndef NUMNODESMETRIC_H
#define NUMNODESMETRIC_H

#include "MetricMethod.h"

template<class MPTraits>
class NumNodesMetric : public MetricMethod<MPTraits> {
  public:
    NumNodesMetric(){
      this->SetName("NumNodesMetric");
    }

    NumNodesMetric(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) : MetricMethod<MPTraits>(_problem, _node){
      this->SetName("NumNodesMetric");
    }

    virtual ~NumNodesMetric(){}

    virtual double operator()(){
      return this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_vertices();
    }
};

#endif
