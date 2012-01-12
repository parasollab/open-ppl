#include "DistanceMetrics.h"
#include "Environment.h"


DistanceMetric::
~DistanceMetric() 
{}


shared_ptr<DistanceMetricMethod>
DistanceMetric::
GetDMMethod(string in_strLabel) 
{
  return ElementSet<DistanceMetricMethod>::GetElement(in_strLabel);
}


void 
DistanceMetric::
AddDMMethod(string in_strLabel, DistanceMetricPointer in_ptr) 
{
  ElementSet<DistanceMetricMethod>::AddElement(in_strLabel, in_ptr);
}


void 
DistanceMetric::
PrintOptions(ostream& out_os) const 
{ 
  out_os << "  Distance Metrics" << endl;
  for(map<string, shared_ptr<DistanceMetricMethod> >::const_iterator M = ElementsBegin(); M != ElementsEnd(); ++M)
    out_os <<"\t\"" << M->first << "\" (" << M->second->GetName() << ")" << endl;
}




