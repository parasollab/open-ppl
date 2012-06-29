#include "Metrics.h"

Metric::~Metric() {}

shared_ptr<MetricsMethod> Metric::GetMethod(string _label) {
  return ElementSet<MetricsMethod>::GetElement(_label);
}

void Metric::PrintOptions(ostream& _os) const {
  _os << "Metrics methods available : " << endl;
  for(map<string, shared_ptr<MetricsMethod> >::const_iterator M = ElementsBegin(); M != ElementsEnd(); ++M)
    _os << "\t\"" << M->first << "\" (" << M->second->GetName() << ")" << endl;
}
