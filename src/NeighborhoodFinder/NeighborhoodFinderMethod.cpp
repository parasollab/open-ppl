#include "NeighborhoodFinderMethod.hpp"
#include "DistanceMetrics.h"

NeighborhoodFinderMethod::
NeighborhoodFinderMethod(shared_ptr<DistanceMetricMethod> _dm) : m_total_time(0.0), m_query_time(0.0), m_construction_time(0.0), m_num_queries(0), dmm(_dm) {}


NeighborhoodFinderMethod::
NeighborhoodFinderMethod(std::string in_strLabel, XMLNodeReader& in_Node, MPProblem* in_pProblem) 
 : LabeledObject(in_strLabel), m_total_time(0.0), m_query_time(0.0), m_construction_time(0.0), m_num_queries(0) 
{ 
  std::string dm_label = in_Node.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");
  dmm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label);
}
  
