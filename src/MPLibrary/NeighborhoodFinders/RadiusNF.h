#ifndef PMPL_RADIUS_NF_H_
#define PMPL_RADIUS_NF_H_

#include "NeighborhoodFinderMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Find all roadmap nodes within a given radius of a query configuration.
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
class RadiusNF: public NeighborhoodFinderMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType      RoadmapType;
    typedef typename RoadmapType::VID               VID;
    typedef typename RoadmapType::VertexSet         VertexSet;
    typedef typename MPBaseObject::GroupRoadmapType GroupRoadmapType;
    typedef typename MPBaseObject::GroupCfgType     GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod::Type;
    using typename NeighborhoodFinderMethod::OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    RadiusNF();

    RadiusNF(XMLNode& _node);

    virtual ~RadiusNF() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinderMethod Overrides
    ///@{

    virtual void FindNeighbors(RoadmapType* const _r, const Cfg& _cfg,
        const VertexSet& _candidates, OutputIterator _out) override;

    virtual void FindNeighbors(GroupRoadmapType* const _r,
        const GroupCfgType& _cfg, const VertexSet& _candidates,
        OutputIterator _out) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    /// Use the nearest node if no nodes are found within the radius.
    bool m_useFallback{false};

    ///@}

};

#endif
