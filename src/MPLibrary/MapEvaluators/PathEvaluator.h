#ifndef PMPL_PATH_EVALUATOR_H
#define PMPL_PATH_EVALUATOR_H

#include "ConfigurationSpace/Path.h"
#include "MapEvaluatorMethod.h"
#include <string>

////////////////////////////////////////////////////////////////////////////////
/// This class calculates various metrics for the path found by another MapEval.
/// Note: this class does not run Dijkstra's.  
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class PathEvaluator : public MapEvaluatorMethod<MPTraits> {

    public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType          RoadmapType;
    typedef typename RoadmapType::VID               VID;
    typedef typename MPTraits::CfgType              CfgType;
    typedef typename MPTraits::Path                 Path;


    ///@}
    ///@name Construction
    ///@{

    PathEvaluator();

    PathEvaluator(XMLNode& _node);

    virtual ~PathEvaluator() = default;

    ///@}

    ///@name MPBaseObject Interface
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluatorMethod Interface
    ///@{

    virtual bool operator()() override;

    ///@}

    protected:

    private:

    /// Returns a vector of length 3:
    /// [0] is the min clearance
    /// [1] is the max clearance
    /// [2] is the average clearance.  
    std::vector<double> GetClearanceStats(const Path* path);

    /// Returns the total path length (using the distance metric).
    double GetPathLength(const Path* path);

    /// Variables fill by XML:
    std::string m_cuLabel{}; // Clearance Utility (MPTool)
    std::string m_ievcLabel{}; // Intermediates Edge Validity Checker
    std::string m_dmLabel{}; // Distance Metric 

    // Why copy/paste when you could write a bunch of helper functions instead?
    // Each of these functions adds (key, value) to this's stat class
    // and prints them to std::cout. 
    // You can find statistics in the .stat output file. 
    void AddToStats(std::string _key, double _value);
    inline void AddToStats(std::string _key, int _value) { AddToStats(_key, (double)_value); };
    inline void AddToStats(std::string _key, bool _value) { AddToStats(_key, (double)_value); };
    inline void AddToStats(std::string _key, size_t _value) { AddToStats(_key, (double)_value); };


};
/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
PathEvaluator<MPTraits>::
PathEvaluator() : MapEvaluatorMethod<MPTraits>(){
  this->SetName("PathEvaluator");
}


template <typename MPTraits>
PathEvaluator<MPTraits>::
PathEvaluator(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("PathEvaluator");

  m_cuLabel = _node.Read("cuLabel", false, "", "Clearance Utility label");
  m_ievcLabel = _node.Read("ievcLabel", false, "", "Intermediate Edge VC label");
  m_dmLabel = _node.Read("dmLabel", false, "", "Distance Metric label");

}

/*------------------------- MPBaseObject Interface ---------------------------*/

template <typename MPTraits>
void
PathEvaluator<MPTraits>::
Initialize() {
  if(this->m_debug)
    std::cout << "PathEvaluator::Initialize()" << std::endl;
}

/*---------------------- MapEvaluatorMethod Interface ------------------------*/

template <typename MPTraits>
bool
PathEvaluator<MPTraits>::
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
        ClearanceUtility<MPTraits>* cu = 
            this->GetMPTools()->GetClearanceUtility(m_cuLabel);
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

template <typename MPTraits>
double
PathEvaluator<MPTraits>::
GetPathLength(const Path* _path) {  
  assert(!m_dmLabel.empty());

  auto r = this->GetRoadmap();
  auto dm = this->GetDistanceMetric(m_dmLabel);
  auto vids = _path->VIDs();

  double dist = 0;
  for(auto start = vids.begin(); start + 1 < vids.end(); ++start) {
      auto cfg1 = r->GetVertex(*start);
      auto cfg2 = r->GetVertex(*(start+1));
      
      dist += dm->Distance(cfg1, cfg2);
  }

  return dist;
}

template <typename MPTraits>
std::vector<double>
PathEvaluator<MPTraits>::
GetClearanceStats(const Path* _path) {  
  assert(!m_ievcLabel.empty());

  auto vc = this->GetEdgeValidityChecker(m_ievcLabel);
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
    
template <typename MPTraits>
void
PathEvaluator<MPTraits>::
AddToStats(std::string _key, double _value) {
    std::string statKey = "PathEvaluator::" + _key;
    this->GetStatClass()->SetStat(statKey, _value);
    std::cout << statKey << "\t" << _value << std::endl;
} 

#endif
