#ifndef NUMEDGESMETRIC_H
#define NUMEDGESMETRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Metrics
/// @brief Get number of edges in roadmap.
/// @tparam MPTraits Motion planning universe
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class NumEdgesMetric : public MetricMethod<MPTraits> {
  public:
    NumEdgesMetric(){
      this->SetName("NumEdgesMetric");
    }

    NumEdgesMetric(typename MPTraits::MPProblemType* _problem, XMLNode& _node)
      : MetricMethod<MPTraits>(_problem, _node) {
      this->SetName("NumEdgesMetric");
    }

    virtual ~NumEdgesMetric(){}

    virtual double operator()(){
      return this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_edges();
    }
};

#endif
