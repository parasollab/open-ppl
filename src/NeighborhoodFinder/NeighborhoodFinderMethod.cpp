#include "NeighborhoodFinderMethod.hpp"
#include "DistanceMetrics.h"
#include "MetricUtils.h"
#include "MPProblem.h"
#include "MPRegion.h"
NeighborhoodFinderMethod::
NeighborhoodFinderMethod(shared_ptr<DistanceMetricMethod> _dm, string _label, MPProblem* _problem) : MPBaseObject(_problem, _label),
                                                  m_total_time(0.0), m_query_time(0.0), m_construction_time(0.0), m_num_queries(0), dmm(_dm) {}


NeighborhoodFinderMethod::
NeighborhoodFinderMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) 
 : MPBaseObject(in_Node, in_pProblem), m_total_time(0.0), m_query_time(0.0), m_construction_time(0.0), m_num_queries(0) 
{ 
  std::string dm_label = in_Node.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");
  dmm = in_pProblem->GetDistanceMetric()->GetMethod(dm_label);
}


NeighborhoodFinderMethod::
NeighborhoodFinderMethod() : m_total_time(0.0), m_query_time(0.0), m_construction_time(0.0), m_num_queries(0) { };

void 
NeighborhoodFinderMethod::
StartTotalTime(){  
    GetMPProblem()->GetMPRegion(0)->GetStatClass()->ClearClock(this->GetName());
    GetMPProblem()->GetMPRegion(0)->GetStatClass()->StartClock(this->GetName());    
}
 
void 
NeighborhoodFinderMethod::
EndTotalTime(){
    GetMPProblem()->GetMPRegion(0)->GetStatClass()->StopClock(this->GetName());
    m_total_time += GetMPProblem()->GetMPRegion(0)->GetStatClass()->GetSeconds(this->GetName());
}
  
void 
NeighborhoodFinderMethod::
StartQueryTime(){
    GetMPProblem()->GetMPRegion(0)->GetStatClass()->ClearClock(this->GetName());
    GetMPProblem()->GetMPRegion(0)->GetStatClass()->StartClock(this->GetName()); 
}
  
void 
NeighborhoodFinderMethod::
EndQueryTime(){
    GetMPProblem()->GetMPRegion(0)->GetStatClass()->StopClock(this->GetName());
    m_query_time +=GetMPProblem()->GetMPRegion(0)->GetStatClass()->GetSeconds(this->GetName());
}

void 
NeighborhoodFinderMethod::
StartConstructionTime(){  
    GetMPProblem()->GetMPRegion(0)->GetStatClass()->ClearClock(this->GetName());
    GetMPProblem()->GetMPRegion(0)->GetStatClass()->StartClock(this->GetName());    
}
  
void 
NeighborhoodFinderMethod::
EndConstructionTime(){
    GetMPProblem()->GetMPRegion(0)->GetStatClass()->StopClock(this->GetName());
    m_construction_time += GetMPProblem()->GetMPRegion(0)->GetStatClass()->GetSeconds(this->GetName());
}
