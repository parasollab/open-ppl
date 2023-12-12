#ifndef PMPL_NEIGHBORHOOD_CONNECTOR_H_
#define PMPL_NEIGHBORHOOD_CONNECTOR_H_

#include "ConnectorMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Connect nearby neighbors together. In this method, the 'second set' of
/// vertices referred to by ConnectorMethod is determined by a nearest neighbors
/// method.
///
/// Connect nodes in map to their neighbors. The following algorithm is used:
/// - for each node, cfg1, in roadmap
///     - find neighbors N for cfg1
///     - lp is a local planner
///     - for each node cfg2 in N and numFailures < m_fail
///         - test lp.IsConnected(cfg1, cfg2)
///         - if connected, add this edge to map, _r.
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
class NeighborhoodConnector: virtual public ConnectorMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType       RoadmapType;
    typedef typename MPBaseObject::GroupCfgType      GroupCfgType;
    typedef typename MPBaseObject::GroupRoadmapType  GroupRoadmapType;
    typedef typename RoadmapType::VID            VID;
    typedef typename RoadmapType::VertexSet      VertexSet;

    ///@}
    ///@name Local Types
    ///@{

    template <typename AbstractRoadmapType>
    using OutputIterator = typename ConnectorMethod::template
                           OutputIterator<AbstractRoadmapType>;

    ///@}
    ///@name Construction
    ///@{

    NeighborhoodConnector();

    NeighborhoodConnector(XMLNode& _node);

    virtual ~NeighborhoodConnector() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name ConnectorMethod Overrides
    ///@{

    virtual void ConnectImpl(RoadmapType* const _r, const VID _source,
        const VertexSet* const _targetSet = nullptr,
        OutputIterator<RoadmapType>* const _collision = nullptr) override;

    virtual void ConnectImpl(GroupRoadmapType* const _r, const VID _source,
        const VertexSet* const _targetSet = nullptr,
        OutputIterator<GroupRoadmapType>* const _collision = nullptr) override;

    using ConnectorMethod::m_neighborBuffer;

    ///@}
    ///@name Internal State
    ///@{

    std::string m_nfLabel;   ///< NeighborhoodFinder for selecting connections.

    ///@}

};

#endif
