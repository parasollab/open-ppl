#include "NumNodesMetric.h"
#include "MPRegion.h"

NumNodesMetric::NumNodesMetric() {
  this->SetName("NumNodesMetric");
}

NumNodesMetric::NumNodesMetric(XMLNodeReader& _node, MPProblem* _problem)
  : MetricsMethod(_node, _problem) {
    this->SetName("NumNodesMetric");
    if(m_debug) PrintOptions(cout);
}

NumNodesMetric::~NumNodesMetric() {
}

void NumNodesMetric::PrintOptions(ostream& _os) {
  _os << "Number of nodes in roadmap." << endl;
}

double NumNodesMetric::operator()(int _regionID) {
  return (double)(GetMPProblem()->GetMPRegion(_regionID)->GetRoadmap()->m_pRoadmap->get_num_vertices());
}
