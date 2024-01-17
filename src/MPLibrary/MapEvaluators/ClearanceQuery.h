#ifndef PMPL_CLEARANCE_QUERY_H
#define PMPL_CLEARANCE_QUERY_H

#include "QueryMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Finds path using weighted clearance rather than path length. 
/// 
/// @note We use 1/clearance as path weights; this can be changed as programmer 
///   sees fit in StaticPathWeight. 
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
class ClearanceQuery : virtual public QueryMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType      RoadmapType;
    typedef typename RoadmapType::VID               VID;
    typedef typename RoadmapType::EdgeID            EdgeID;
    typedef std::unordered_set<size_t>              VIDSet;
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

    std::string m_intermediateEdgeVCLabel; // EdgeIntermediate Validity Checker (for weighted clearance)

  private:
    /// "Cache" edges so we don't need to recheck them using the collision checker. 
    void CacheEdge(VID _u, VID _v, double _value) const;

    std::map<std::pair<VID, VID>, double>* m_cachedEdges; // map that acts as the "cache"; filled in CacheEdge() 
};

#endif
