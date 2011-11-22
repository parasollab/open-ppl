#include "DistanceMetrics.h"
#include "Environment.h"


DistanceMetric::
~DistanceMetric() 
{}


shared_ptr<DistanceMetricMethod>
DistanceMetric::
GetDMMethod(string in_strLabel) 
{
  DistanceMetricPointer to_return = element_set<DistanceMetricMethod>::get_element(in_strLabel);
  if(to_return.get() == NULL) 
  {
    cerr << "\n\n\tERROR in GetDMMethod, requesting dm method labeled \"" << in_strLabel << "\" which does not exist in the list\n";
    for(map<string, shared_ptr<DistanceMetricMethod> >::const_iterator M = elements_begin(); M != elements_end(); ++M)
      cerr <<"\t\"" << M->first << "\" (" << M->second->GetName() << ")" << endl;
    cerr << "exiting.\n";
    exit(-1);
  }
  return to_return;
}


void 
DistanceMetric::
AddDMMethod(string in_strLabel, DistanceMetricPointer in_ptr) 
{
  element_set<DistanceMetricMethod>::add_element(in_strLabel, in_ptr);
}


void 
DistanceMetric::
PrintOptions(ostream& out_os) const 
{ 
  out_os << "  Distance Metrics" << endl;
  for(map<string, shared_ptr<DistanceMetricMethod> >::const_iterator M = elements_begin(); M != elements_end(); ++M)
    out_os <<"\t\"" << M->first << "\" (" << M->second->GetName() << ")" << endl;
}




