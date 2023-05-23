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

    ///@}
    ///@name QueryMethod Overrides
    ///@{

    /// Set an alternate path weight function to use when searching the roadmap
    /// @param _f The path weight function to use.
    virtual void SetPathWeightFunction(SSSPPathWeightFunction<RoadmapType> _f)
        override;

    ///@}

  protected:

    ///@name QueryMethod Overrides
    ///@{

    /// Reset the path and list of undiscovered goals
    /// @param _r The roadmap to use.
    virtual void Reset(RoadmapType* const _r) override;

    virtual double StaticPathWeight(
        typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const
        override;

    virtual double DynamicPathWeight(
        typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const
        override;

    ///@}

    std::set<EI> m_seenEdges;

    std::string m_edgeIntermediateVCLabel;
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ClearanceQuery<MPTraits>::
ClearanceQuery() : QueryMethod<MPTraits>() {
  this->SetName("ClearanceQuery");
}


template <typename MPTraits>
ClearanceQuery<MPTraits>::
ClearanceQuery(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node), QueryMethod<MPTraits>(_node) {
  this->SetName("ClearanceQuery");

  m_edgeIntermediateVCLabel = _node.Read("eivcLabel", true, "", "the edge intermediate VC label for weighted clearance checking");


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
  m_seenEdges.clear();
}


template <typename MPTraits>
double
ClearanceQuery<MPTraits>::
StaticPathWeight(EI& _ei,
    const double _sourceDistance, const double _targetDistance) const {

  VID source = _ei->source();
  VID target = _ei->target();
  std::cout << source << " " << target << std::endl;
  double edgeWeight = -1;

  // if (m_seenEdges.count(_ei) > 0) {
  //   edgeWeight = _ei->property().GetWeight();
  // } else {
    auto vc = this->GetEdgeValidityChecker(m_edgeIntermediateVCLabel);
    edgeWeight = vc->AssignClearanceWeight(source, target);
  //   m_seenEdges.insert(_ei);
  // }

   

  return _sourceDistance + edgeWeight;
}


/*----------------------------------------------------------------------------*/

#endif
