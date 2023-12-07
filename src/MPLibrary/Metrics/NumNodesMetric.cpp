#include "NumNodesMetric.h"

/*------------------------------ Construction --------------------------------*/

NumNodesMetric::
NumNodesMetric() {
  this->SetName("NumNodesMetric");
}


NumNodesMetric::
NumNodesMetric(XMLNode& _node) : MetricMethod(_node){
  this->SetName("NumNodesMetric");
}

/*---------------------------- Metric Interface ------------------------------*/

double
NumNodesMetric::
operator()() {
  if(this->GetGroupRoadmap())
    return this->GetGroupRoadmap()->Size();
  else
    return this->GetRoadmap()->Size();
}

/*----------------------------------------------------------------------------*/
