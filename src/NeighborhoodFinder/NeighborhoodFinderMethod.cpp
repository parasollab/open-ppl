#include "NeighborhoodFinderMethod.hpp"
#include "DistanceMetrics.h"
#include "MetricUtils.h"

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

ClockClass m_clock_total, m_clock_query, m_clock_cons;

void 
NeighborhoodFinderMethod::
StartTotalTime(){  
    m_clock_total.ClearClock();
    m_clock_total.StartClock("");    
}
 
void 
NeighborhoodFinderMethod::
EndTotalTime(){
    m_clock_total.StopClock();
    m_total_time += m_clock_total.GetSeconds();
}
  
void 
NeighborhoodFinderMethod::
StartQueryTime(){
    m_clock_query.ClearClock();
    m_clock_query.StartClock(""); 
}
  
void 
NeighborhoodFinderMethod::
EndQueryTime(){
    m_clock_query.StopClock();
    m_query_time += m_clock_query.GetSeconds();
}

void 
NeighborhoodFinderMethod::
StartConstructionTime(){  
    m_clock_cons.ClearClock();
    m_clock_cons.StartClock("");    
}
  
void 
NeighborhoodFinderMethod::
EndConstructionTime(){
    m_clock_cons.StopClock();
    m_construction_time += m_clock_cons.GetSeconds();
}
