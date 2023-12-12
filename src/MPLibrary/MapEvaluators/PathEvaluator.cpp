#include "PathEvaluator.h"

#include "MPLibrary/MPLibrary.h"

#include <string>

/*------------------------------ Construction --------------------------------*/

PathEvaluator::
PathEvaluator() : MapEvaluatorMethod(){
  this->SetName("PathEvaluator");
}


PathEvaluator::
PathEvaluator(XMLNode& _node) : MapEvaluatorMethod(_node) {
  this->SetName("PathEvaluator");

  m_cuLabel = _node.Read("cuLabel", false, "", "Clearance Utility label");
  m_ievcLabel = _node.Read("ievcLabel", false, "", "Intermediate Edge VC label");
  m_dmLabel = _node.Read("dmLabel", false, "", "Distance Metric label");

}

/*------------------------- MPBaseObject Interface ---------------------------*/

void
PathEvaluator::
Initialize() {
  if(this->m_debug)
    std::cout << "PathEvaluator::Initialize()" << std::endl;
}

/*---------------------- MapEvaluatorMethod Interface ------------------------*/

bool
PathEvaluator::
operator()() {
    const Path* path = this->GetMPSolution()->GetPath();

    if(!path || path->Empty()) {
        if(this->m_debug)
          std::cout << "Path was not found. No Path evaluation can be done. " << std::endl;
        
        return true;
    }

    // Edge Weight and size
    AddToStats("TotalEdgeWeight", path->Length());
    AddToStats("PathSize", path->Size());

    // Actual Path Length
    if (!m_dmLabel.empty()) {
        AddToStats("Actual Path Length", GetPathLength(path));
    }

    // Clearance (pure)
    if (!m_cuLabel.empty()) {
        ClearanceUtility* cu = 
            this->GetMPLibrary()->GetMPTools()->GetClearanceUtility(m_cuLabel);
        std::vector<VID> pathVIDs = this->GetPath()->VIDs();
        ClearanceStats cs = cu->PathClearance(pathVIDs);
        AddToStats("AvgClearance", cs.m_avg);
        AddToStats("MinClearance", cs.m_min);
        AddToStats("MaxClearance", cs.m_max);
    } 

    // Risk-weighted path clearance.
    if (!m_ievcLabel.empty()) {
        auto clearances = GetClearanceStats(path);
        AddToStats("RiskWeightedMinClearance", clearances[0]);
        AddToStats("RiskWeightedMaxClearance", clearances[1]);
        AddToStats("RiskWeightedAvgClearance", clearances[2]);
    }

    return true;
}

/*--------------------------- Helper Methods ---------------------------------*/

double
PathEvaluator::
GetPathLength(const Path* _path) {  
  assert(!m_dmLabel.empty());

  auto r = this->GetRoadmap();
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
  auto vids = _path->VIDs();

  double dist = 0;
  for(auto start = vids.begin(); start + 1 < vids.end(); ++start) {
      auto cfg1 = r->GetVertex(*start);
      auto cfg2 = r->GetVertex(*(start+1));
      
      dist += dm->Distance(cfg1, cfg2);
  }

  return dist;
}


std::vector<double>
PathEvaluator::
GetClearanceStats(const Path* _path) {  
  assert(!m_ievcLabel.empty());

  auto vc = this->GetMPLibrary()->GetEdgeValidityChecker(m_ievcLabel);
  auto vids = _path->VIDs();

  double minClearance = MAX_DBL;
  double maxClearance = 0;
  double clearanceSum = 0;
  for(auto start = vids.begin(); start + 1 < vids.end(); ++start) {
    double clearance = vc->EdgeWeightedClearance(*start, *(start+1));

    clearanceSum += clearance;
    minClearance = (clearance < minClearance) ? clearance : minClearance;
    maxClearance = (clearance > maxClearance) ? clearance : maxClearance;
  }

  return {minClearance, maxClearance, clearanceSum/_path->Size()};
}

/*-------------------- Add to Stats Methods --------------------------*/
    
void
PathEvaluator::
AddToStats(std::string _key, double _value) {
    std::string statKey = "PathEvaluator::" + _key;
    this->GetStatClass()->SetStat(statKey, _value);
    std::cout << statKey << "\t" << _value << std::endl;
} 
