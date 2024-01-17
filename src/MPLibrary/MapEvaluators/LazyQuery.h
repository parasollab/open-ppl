#ifndef PMPL_LAZY_QUERY_H_
#define PMPL_LAZY_QUERY_H_

#include "QueryMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// First assumes all nodes and edges are valid, then checks for validity of the
/// nodes/edges used in the path.
///
/// Reference:
///   Robert Bohlin and Lydia E. Kavraki. "Path Planning Using Lazy PRM".
///   ICRA 2000.
///
/// @note Node enhancement does not work like in the paper. Here we use a
///       flat gaussian distribution with fixed distance.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
class LazyQuery : virtual public QueryMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType      RoadmapType;
    typedef typename RoadmapType::VID               VID;
    typedef typename RoadmapType::EdgeID            EdgeID;
    typedef std::unordered_set<VID>                 VIDSet;

    ///@}
    ///@name Construction
    ///@{

    LazyQuery();
    LazyQuery(XMLNode& _node);
    virtual ~LazyQuery() = default;

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

    ///@name Internal Types
    ///@{

    typedef std::unordered_set<VID>    VertexSet;
    typedef std::unordered_set<EdgeID> EdgeSet;

    ///@}
    ///@name QueryMethod Overrides
    ///@{

    /// Reset the path and list of undiscovered goals
    /// @param _r The roadmap to use.
    virtual void Reset(RoadmapType* const _r) override;

    virtual bool PerformSubQuery(const VID _start, const VIDSet& _goals)
        override;

    virtual double StaticPathWeight(
        typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const
        override;

    virtual double DynamicPathWeight(
        typename RoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const
        override;

    ///@}
    ///@name Helpers
    ///@{

    /// Checks validity of nodes and edges and deletes any invalid ones.
    /// @return True if the path was valid.
    bool ValidatePath();

    /// Check each vertex and ensure it is valid. Upon discovering an invalid
    /// vertex, delete it and return.
    /// @return True if a vertex was deleted.
    bool PruneInvalidVertices();

    /// Check each edge and ensure it is valid. Upon discovering an invalid edge,
    /// delete it and return.
    /// @return True if an edge was deleted.
    bool PruneInvalidEdges();

    /// Choose a random deleted edge and generate nodes with a gaussian
    /// distribution around the edge's midpoint.
    virtual void NodeEnhance();

    /// Additional handling of invalid vertices.
    /// @param _cfg The invalid configuration to handle.
    virtual void ProcessInvalidNode(const Cfg& _cfg) { }

    /// Invalidate or delete a roadmap configuration according to the deletion
    /// option.
    /// @param _vid The vertex descriptor.
    void InvalidateVertex(const VID _vid);

    /// Invalidate or delete a roadmap edge according to the deletion option.
    /// @param _source The source vertex descriptor.
    /// @param _target The target vertex descriptor.
    void InvalidateEdge(const VID _source, const VID _target);

    ///@}
    ///@name Lazy Invalidation
    ///@{

    /// Set a vertex as invalidated.
    /// @param _vid The vertex descriptor.
    void SetVertexInvalidated(const VID _vid) noexcept;

    /// Check if a vertex is lazily invalidated.
    /// @param _vid The vertex descriptor.
    /// @return     True if _vid is lazily invalidated.
    bool IsVertexInvalidated(const VID _vid) const noexcept;

    /// Check if an edge is lazily invalidated.
    /// @param _eid The edge ID.
    /// @return     True if _eid is lazily invalidated.
    bool IsEdgeInvalidated(const EdgeID _eid) const noexcept;

    /// @overload This version takes the source and target VIDs for an edge.
    /// @param _source The VID of the source vertex.
    /// @param _target The VID of the target vertex.
    /// @return        True if (_source, _target) is lazily invalidated.
    bool IsEdgeInvalidated(const VID _source, const VID _target) const noexcept;

    /// Set an edge as invalidated.
    /// @param _eid The edge ID.
    void SetEdgeInvalidated(const EdgeID _eid) noexcept;

    /// @overload
    /// @param _source  The VID of the source vertex.
    /// @param _target  The VID of the target vertex.
    void SetEdgeInvalidated(const VID _source, const VID _target) noexcept;

    ///@}
    ///@name MP Object Labels
    ///@{

    std::string m_vcLabel;         ///< The lazy validity checker label.
    std::string m_lpLabel;         ///< The lazy local planner label.
    std::string m_enhanceDmLabel;  ///< The distance metric for enhancement.

    std::vector<std::string> m_ncLabels; ///< The connectors for enhancement.

    ///@}
    ///@name Internal State
    ///@{

    bool m_deleteInvalid{true};   ///< Remove invalid vertices from the roadmap?

    std::vector<int> m_resolutions{1}; ///< List of resolution multiples to check.
    size_t m_numEnhance{0};       ///< Number of enhancement nodes to generate.
    double m_d{0};                ///< Gaussian distance for enhancement sampling.

    /// Candidate edges for enhancement sampling.
    std::vector<std::pair<Cfg, Cfg>> m_edges;

    /// Lazy-invalidated vertices.
    std::unordered_map<RoadmapType*, VertexSet> m_invalidVertices;
    /// Lazy-invalidated edges.
    std::unordered_map<RoadmapType*, EdgeSet> m_invalidEdges;

    ///@}

};

#endif
