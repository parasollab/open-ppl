#include "NumEdgesMetric.h"
#include "Roadmap.h"

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

double NumEdgesMetric::operator()() {
  return (double)(GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_edges());
}
