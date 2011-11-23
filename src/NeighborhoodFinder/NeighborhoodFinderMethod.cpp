#include "NeighborhoodFinderMethod.hpp"
#include "DistanceMetrics.h"

NeighborhoodFinderMethod::
NeighborhoodFinderMethod(shared_ptr<DistanceMetricMethod> _dm, string _label, MPProblem* _problem) : MPBaseObject(_problem, _label),
                                                  m_total_time(0.0), m_query_time(0.0), m_construction_time(0.0), m_num_queries(0), dmm(_dm) {}


NeighborhoodFinderMethod::
NeighborhoodFinderMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) 
 : MPBaseObject(in_Node, in_pProblem), m_total_time(0.0), m_query_time(0.0), m_construction_time(0.0), m_num_queries(0) 
{ 
  std::string dm_label = in_Node.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");
  dmm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label);
}


NeighborhoodFinderMethod::
NeighborhoodFinderMethod() : m_total_time(0.0), m_query_time(0.0), m_construction_time(0.0), m_num_queries(0) { };
