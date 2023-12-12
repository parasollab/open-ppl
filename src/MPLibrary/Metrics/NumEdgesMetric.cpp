#include "NumEdgesMetric.h"

/*------------------------------- Construction -------------------------------*/

NumEdgesMetric::
NumEdgesMetric() {
  this->SetName("NumEdgesMetric");
}


NumEdgesMetric::
NumEdgesMetric(XMLNode& _node) : MetricMethod(_node) {
  this->SetName("NumEdgesMetric");
}

/*--------------------------- MetricMethod Interface -------------------------*/

double
NumEdgesMetric::
operator()() {
  if(this->GetGroupTask())
    return this->GetGroupRoadmap()->get_num_edges();
  else
    return this->GetRoadmap()->get_num_edges();
}

/*----------------------------------------------------------------------------*/
