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
    typedef typename MPTraits::Path                    Path;


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

    /// Returns the Min Clearance of the path. 
    double GetMinClearance(const Path* path);

    /// Returns the total path length (using the distance metric).
    double GetPathLength(const Path* path);

    std::string m_cuLabel{};
    std::string m_ievcLabel{};
    std::string m_dmLabel{};

    // Why copy/paste when you could write a bunch of helper functions instead?
    void AddToStats(std::string key, int value);
    void AddToStats(std::string key, double value);
    void AddToStats(std::string key, bool value);
    void AddToStats(std::string key, size_t value);
    void AddToStats(std::string key, std::string value);


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
//   m_scLabel = _node.Read("scLabel", false, "", "Segment Clearance Tool label");
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
        double minClearance = GetMinClearance(path);
        AddToStats("Min Clearance", minClearance);
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
double
PathEvaluator<MPTraits>::
GetMinClearance(const Path* _path) {  
  assert(!m_ievcLabel.empty());

  auto vc = this->GetEdgeValidityChecker(m_ievcLabel);
  auto vids = _path->VIDs();

  double minClearance = MAX_DBL;
  for(auto start = vids.begin(); start + 1 < vids.end(); ++start) {
    double clearance = vc->AssignClearanceWeight(*start, *(start+1));

    if (clearance < minClearance) {
        minClearance = clearance;
    }
  }

  return minClearance;
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

template <typename MPTraits>
void
PathEvaluator<MPTraits>::
AddToStats(std::string _key, int _value) {
    AddToStats(_key, (double)_value);
}

template <typename MPTraits>
void
PathEvaluator<MPTraits>::
AddToStats(std::string _key, size_t _value) {
    AddToStats(_key, (double)_value);
}

template <typename MPTraits>
void
PathEvaluator<MPTraits>::
AddToStats(std::string _key, bool _value) {
    AddToStats(_key, (double)_value);
}


#endif
