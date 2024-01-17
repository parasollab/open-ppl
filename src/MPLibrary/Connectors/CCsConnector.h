#ifndef PMPL_CCS_CONNECTOR_H_
#define PMPL_CCS_CONNECTOR_H_

#include "ConnectorMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Tries to connect the source vertices to targets in other CCs.
///
/// @note If the skip same CC option is set, it will forgo checking the
///       remainder of the targets in a target CC after the first success.
///
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
class CCsConnector: virtual public ConnectorMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType   RoadmapType;
    typedef typename RoadmapType::VID            VID;
    typedef typename RoadmapType::VertexSet      VertexSet;
    typedef typename MPBaseObject::GroupRoadmapType  GroupRoadmapType;

    ///@}
    ///@name Local Types
    ///@{

    template <typename AbstractRoadmapType>
    using OutputIterator = typename ConnectorMethod::template
                           OutputIterator<AbstractRoadmapType>;

    ///@}
    ///@name Construction
    ///@{

    CCsConnector();

    CCsConnector(XMLNode& _node);

    virtual ~CCsConnector() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name ConnectorMethod Overrides
    ///@{

    /// @note If a target set is provided, we only attempt to connect to CCs
    ///       containing at least one of its members.
    
    /// Generate edges with single vertex as source
    /// @param _r Roadmap to connect
    /// @param _source Source vertex to connect
    /// @param _targetSet The set of target vertices, set to null for full roadmap
    /// @param _collision Output iterator for collisons
    virtual void ConnectImpl(RoadmapType* const _r, const VID _source,
        const VertexSet* const _targetSet = nullptr,
        OutputIterator<RoadmapType>* const _collision = nullptr) override;

    using ConnectorMethod::m_neighborBuffer;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_nfLabel;  ///< The neighborhood finder for nearest nodes.

    ///@}

};

#endif
