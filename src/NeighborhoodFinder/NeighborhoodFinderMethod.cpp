#include "NeighborhoodFinderMethod.hpp"
#include "DistanceMetrics.h"
#include "MetricUtils.h"
#include "MPProblem.h"
#include "MPRegion.h"

NeighborhoodFinderMethod::NeighborhoodFinderMethod(string _dmLabel, string _label, MPProblem* _problem) 
  : MPBaseObject(_problem, _label), m_dmLabel(_dmLabel), m_fromRDMPVersion(false) {}


NeighborhoodFinderMethod::NeighborhoodFinderMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) 
  : MPBaseObject(in_Node, in_pProblem), m_fromRDMPVersion(false){ 
    m_dmLabel = in_Node.stringXMLParameter("dmMethod", true, "default", "Distance Metric Method");
  }

shared_ptr<DistanceMetricMethod> 
NeighborhoodFinderMethod::GetDMMethod() const {
  return GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmLabel);
}

double
NeighborhoodFinderMethod::GetTotalTime() const{
  return GetMPProblem()->GetMPRegion(0)->GetStatClass()->GetSeconds(this->GetNameAndLabel()+"::Total");
}

double 
NeighborhoodFinderMethod::GetQueryTime() const{
  return GetMPProblem()->GetMPRegion(0)->GetStatClass()->GetSeconds(this->GetNameAndLabel()+"::Query");
}

double 
NeighborhoodFinderMethod::GetConstructionTime() const{
  return GetMPProblem()->GetMPRegion(0)->GetStatClass()->GetSeconds(this->GetNameAndLabel()+"::Construction");
}

size_t 
NeighborhoodFinderMethod::GetNumQueries() const{
  return GetMPProblem()->GetMPRegion(0)->GetStatClass()->GetNFStat(this->GetNameAndLabel()+"::NumQueries");
}

void 
NeighborhoodFinderMethod::StartTotalTime(){  
  GetMPProblem()->GetMPRegion(0)->GetStatClass()->StartClock(this->GetNameAndLabel()+"::Total");    
}

void 
NeighborhoodFinderMethod::EndTotalTime(){
  GetMPProblem()->GetMPRegion(0)->GetStatClass()->StopClock(this->GetNameAndLabel()+"::Total");
}

void 
NeighborhoodFinderMethod::StartQueryTime(){
  GetMPProblem()->GetMPRegion(0)->GetStatClass()->StartClock(this->GetNameAndLabel()+"::Query");    
}

void 
NeighborhoodFinderMethod::EndQueryTime(){
  GetMPProblem()->GetMPRegion(0)->GetStatClass()->StopClock(this->GetNameAndLabel()+"::Query");
}

void 
NeighborhoodFinderMethod::StartConstructionTime(){
  GetMPProblem()->GetMPRegion(0)->GetStatClass()->StartClock(this->GetNameAndLabel()+"::Construction");    
}

void 
NeighborhoodFinderMethod::EndConstructionTime(){
  GetMPProblem()->GetMPRegion(0)->GetStatClass()->StopClock(this->GetNameAndLabel()+"::Construction");
}

void 
NeighborhoodFinderMethod::IncrementNumQueries(){
  GetMPProblem()->GetMPRegion(0)->GetStatClass()->IncNFStat(this->GetNameAndLabel()+"::NumQueries");
}
