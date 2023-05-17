#ifndef PMPL_PATH_EVALUATOR_H
#define PMPL_PATH_EVALUATOR_H

#include "ConfigurationSpace/Path.h"
#include "MapEvaluatorMethod.h"
#include <string>

////////////////////////////////////////////////////////////////////////////////
/// A stop-watch like evaluator that returns false after a set amount of time.
///
/// This class maintains a separate clock for each instance. For a global clock
/// use ConditionalEvaluator with the TimeMetric.
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

    std::string m_cuLabel{};

    // Why copy/paste when you could write a million helper functions instead?
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
//   m_scLabel = _node.Read("scLabel", false, "", "Segment Clearance Tool label");
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
    auto path = this->GetMPSolution()->GetPath();

    if(!path || path->Empty()) {
        if(this->m_debug)
        std::cout << "Path was not found. No Path evaluation can be done. " << std::endl;
        
        return true;
    }

    // Path length and size
    AddToStats("PathLength", path->Length());
    AddToStats("PathSize", path->Size());

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

    // TODO: Risk-weighted path clearance.

    // TODO: Sensitivity to rotation.

    // TODO: Min Clearance on ideal rotation.

    // TODO: x-sectional min/average width.


    return true;

}

/*--------------------------- Helper Methods ---------------------------------*/

template <typename MPTraits>
void
PathEvaluator<MPTraits>::
AddToStats(std::string key, double value) {
    std::string statKey = "PathEvaluator::" + key;
    this->GetStatClass()->SetStat(statKey, value);
    std::cout << statKey << "\t" << value << std::endl;
} 

template <typename MPTraits>
void
PathEvaluator<MPTraits>::
AddToStats(std::string key, int value) {
    AddToStats(key, (double)value);
}

template <typename MPTraits>
void
PathEvaluator<MPTraits>::
AddToStats(std::string key, size_t value) {
    AddToStats(key, (double)value);
}

template <typename MPTraits>
void
PathEvaluator<MPTraits>::
AddToStats(std::string key, bool value) {
    AddToStats(key, (double)value);
}


#endif