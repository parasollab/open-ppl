#include "NumEdgesMetric.h"
#include "MPRegion.h"

NumEdgesMetric::NumEdgesMetric() {
  this->SetName("NumEdgesMetric");
}

NumEdgesMetric::NumEdgesMetric(XMLNodeReader& _node, MPProblem* _problem)
  : MetricsMethod(_node, _problem) {
    this->SetName("NumEdgesMetric");
    if(m_debug) PrintOptions(cout);
}

NumEdgesMetric::~NumEdgesMetric() {
}

void NumEdgesMetric::PrintOptions(ostream& _os) {
  _os << "Number of edges in roadmap." << endl;
}

double NumEdgesMetric::operator()(int _regionID) {
  return (double)(GetMPProblem()->GetMPRegion(_regionID)->GetRoadmap()->m_pRoadmap->get_num_edges());
}
