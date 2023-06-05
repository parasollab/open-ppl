#ifndef PMPL_CLEARANCE_QUERY_H
#define PMPL_CLEARANCE_QUERY_H

#include "QueryMethod.h"
#include <map>
#include <set>

template <typename MPTraits>
class ClearanceQuery : virtual public QueryMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType              CfgType;
    typedef typename MPTraits::RoadmapType          RoadmapType;
    typedef typename RoadmapType::VID               VID;
    typedef typename RoadmapType::EdgeID            EdgeID;
    typedef typename MPTraits::GoalTracker          GoalTracker;
    typedef typename GoalTracker::VIDSet            VIDSet;
    typedef typename RoadmapType::adj_edge_iterator EI;

    ///@}
    ///@name Construction
    ///@{

    ClearanceQuery();
    ClearanceQuery(XMLNode& _node);
    virtual ~ClearanceQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    virtual void Initialize() override;

  protected:

    ///@name QueryMethod Overrides
    ///@{

    /// Reset the path and list of undiscovered goals
    /// @param _r The roadmap to use.
    virtual void Reset(RoadmapType* const _r) override;

    /// Set the path weights as minimum 1/clearance.
    virtual double StaticPathWeight(
        typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const
        override;

    ///@}

    std::string m_edgeIntermediateVCLabel;

  private:
    void CacheEdge(VID _u, VID _v, double _value) const;

    std::map<std::pair<VID, VID>, double>* m_cachedEdges; // Cache pairs of edges so we don't 
                                                         // need to call Edge Validity checker 
                                                         // on them again. 
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ClearanceQuery<MPTraits>::
ClearanceQuery() : QueryMethod<MPTraits>() {
  this->SetName("ClearanceQuery");
  m_cachedEdges = new std::map<std::pair<VID, VID>, double>;
}


template <typename MPTraits>
ClearanceQuery<MPTraits>::
ClearanceQuery(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node), QueryMethod<MPTraits>(_node) {
  this->SetName("ClearanceQuery");

  m_edgeIntermediateVCLabel = _node.Read("eivcLabel", true, "",
         "the edge intermediate VC label for weighted clearance checking");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
ClearanceQuery<MPTraits>::
Print(std::ostream& _os) const {
  QueryMethod<MPTraits>::Print(_os);
  _os << std::endl;
}


template <typename MPTraits>
void
ClearanceQuery<MPTraits>::
Initialize() {
  QueryMethod<MPTraits>::Initialize();
}

/*--------------------------- QueryMethod Overrides --------------------------*/

template <typename MPTraits>
void
ClearanceQuery<MPTraits>::
Reset(RoadmapType* const _r) {
  QueryMethod<MPTraits>::Reset(_r);
  m_cachedEdges = new std::map<std::pair<VID, VID>, double>;
  m_cachedEdges->clear();
}

template <typename MPTraits>
double
ClearanceQuery<MPTraits>::
StaticPathWeight(EI& _ei, 
  const double _sourceDistance, const double _targetDistance) const {
    auto vc = this->GetEdgeValidityChecker(m_edgeIntermediateVCLabel);

    VID source = _ei->source();
    VID target = _ei->target();

    // This calculation can be any other weight as user desires.
    double clearance = 0;

    std::pair<VID, VID> st(source, target);
    std::pair<VID, VID> ts(target, source);
    if (m_cachedEdges->count(st) == 1 || m_cachedEdges->count(ts) == 1) {
        if (this->m_debug) {
            std::cout << "Edge " << source << ", " << target << " already cached." <<std::endl;
        }
        
        clearance = m_cachedEdges->at(st);
    } else {
        clearance = vc->EdgeWeightedClearance(source, target); 
		    CacheEdge(source, target, clearance);
    }
    
    double edgeWeight = 1./clearance;

    if (clearance == 0) {
        edgeWeight = MAX_DBL;       
    }

    if(this->m_debug) {
        std::cout << "Edge weighted clearance " << clearance;
        std::cout << "; vids " << source << " " << target << std::endl;
    }

    return std::max(_sourceDistance, edgeWeight);
}

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
void
ClearanceQuery<MPTraits>::
CacheEdge(VID _u, VID _v, double _value) const {
    std::pair<VID, VID> uv (_u, _v);
    std::pair<VID, VID> vu (_v, _u);

    m_cachedEdges->emplace(uv, _value);
    m_cachedEdges->emplace(vu, _value);

    if (this->m_debug) {
        std::cout << "Cached edge " << _u << ", " << _v << ". " ;
        std::cout << m_cachedEdges->size() << " edges cached." << std::endl;
    }

}

#endif
