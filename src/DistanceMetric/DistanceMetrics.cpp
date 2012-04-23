#include "DistanceMetrics.h"
#include "Environment.h"


DistanceMetric::~DistanceMetric() {}

shared_ptr<DistanceMetricMethod> DistanceMetric::GetMethod(string _label) {
  return ElementSet<DistanceMetricMethod>::GetElement(_label);
}

void 
DistanceMetric::AddMethod(string const& _label, DistanceMetricPointer _dmm){
  ElementSet<DistanceMetricMethod>::AddElement(_label, _dmm);
}

void DistanceMetric::PrintOptions(ostream& _os) const { 
	_os << "  Distance Metrics methods available:" << endl;
  for(map<string, shared_ptr<DistanceMetricMethod> >::const_iterator M = ElementsBegin(); M != ElementsEnd(); ++M)
    _os <<"\t\"" << M->first << "\" (" << M->second->GetName() << ")" << endl;
}




