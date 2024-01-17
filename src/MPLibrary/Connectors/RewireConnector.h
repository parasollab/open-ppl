#ifndef PMPL_REWIRE_CONNECTOR_H_
#define PMPL_REWIRE_CONNECTOR_H_

#include "ConnectorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Re-wires a tree for optimal RRT planners. This only makes sense for
/// tree-like planners. It will change the structure of the tree; ensure that
/// your algorithm can handle this before using.
///
/// Reference:
///   Sertac Karaman and Emilio Frazzoli. "Sampling-based algorithms for optimal
///   motion planning". IJRR 2011.
///
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
class RewireConnector : virtual public ConnectorMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType      GroupCfgType;
    typedef typename MPBaseObject::WeightType        WeightType;
    typedef typename MPBaseObject::GroupWeightType   GroupWeightType;
    typedef typename MPBaseObject::RoadmapType       RoadmapType;
    typedef typename MPBaseObject::GroupRoadmapType  GroupRoadmapType;
    typedef typename RoadmapType::VID            VID;
    typedef typename RoadmapType::VertexSet      VertexSet;

    ///@}
    ///@name Local Types
    ///@{

    /// An aggregate for returning the results of a rewiring check.
    struct RewireTestOutput {
      bool               passed{false}; ///< Should we use this rewiring?
      LPOutput lpo;           ///< The generated local plan.
      double             cost{0};       ///< New total cost to the rewired node.

      RewireTestOutput(const bool _passed = false,
          LPOutput&& _lpo = LPOutput(),
          const double _cost = 0) :
          passed(_passed),
          lpo(_lpo),
          cost(_cost)
      { }
    };

    /// An aggregate for returning the results of a group rewiring check.
    struct GroupRewireTestOutput {
      bool               passed{false}; ///< Should we use this rewiring?
      GroupLPOutput lpo;      ///< The generated local plan.
      double             cost{0};       ///< New total cost to the rewired node.

      GroupRewireTestOutput(const bool _passed = false,
          GroupLPOutput&& _lpo = GroupLPOutput(),
          const double _cost = 0) :
          passed(_passed),
          lpo(_lpo),
          cost(_cost)
      { }
    };

    template <typename AbstractRoadmapType>
    using OutputIterator = typename ConnectorMethod::template
                           OutputIterator<AbstractRoadmapType>;

    ///@}
    ///@name Construction
    ///@{

    RewireConnector();

    RewireConnector(XMLNode& _node);

    virtual ~RewireConnector() = default;

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

  private:

    ///@name Helpers
    ///@{

    /// Attempt to rewire a vertex through its nearest neighbors if doing so
    /// results in a shorter path to the nearest root vertex.
    /// @param _r         The containing roadmap.
    /// @param _vid       The vertex whos shortest path may be re-routed.
    /// @param _neighbors The set of potential new parents for _vid.
    /// @param _collision Optional output iterator for collisions.
    void RewireVertex(RoadmapType* const _r, const VID _vid,
        const std::vector<Neighbor>& _neighbors,
        OutputIterator<RoadmapType>* const _collision);

    /// @overload
    void RewireVertex(GroupRoadmapType* const _r, const VID _vid,
        const std::vector<Neighbor>& _neighbors,
        OutputIterator<GroupRoadmapType>* const _collision);

    /// Attempt to rewire the nearest-neighbors of a vertex _vid through itself
    /// if doing so results in a shorter path to the nearest root vertex.
    /// @param _r         The containing roadmap.
    /// @param _vid       The vertex whos shortest path may be re-routed.
    /// @param _neighbors The set of potential new parents for _vid.
    /// @param _collision Output iterator for collisions.
    void RewireNeighbors(RoadmapType* const _r, const VID _vid,
        const std::vector<Neighbor>& _neighbors,
        OutputIterator<RoadmapType>* const _collision);

    /// @overload
    void RewireNeighbors(GroupRoadmapType* const _r, const VID _vid,
        const std::vector<Neighbor>& _neighbors,
        OutputIterator<GroupRoadmapType>* const _collision);

    /// Check if a vertex should be rewired through a new parent.
    /// @param _r                   The containing roadmap.
    /// @param _vid                 The vertex which may be rewired.
    /// @param _currentParent       The vertex's current parent.
    /// @param _currentCost         The vertex's current cost.
    /// @param _potentialParent     The potential parent to check.
    /// @param _potentialParentCost The cost to reach the potential parent.
    /// @param _collision           Output iterator for collisions.
    /// @return A rewire test output indicating success and generated lp/cost.
    RewireTestOutput RewireTest(RoadmapType* const _r, const VID _vid,
        const VID _currentParent, const double _currentCost,
        const VID _potentialParent, const double _potentialParentCost,
        OutputIterator<RoadmapType>* const _collision) noexcept;

    /// @overload
    GroupRewireTestOutput RewireTest(GroupRoadmapType* const _r, const VID _vid,
        const VID _currentParent, const double _currentCost,
        const VID _potentialParent, const double _potentialParentCost,
        OutputIterator<GroupRoadmapType>* const _collision) noexcept;

    /// Trace the path from one vertex to another through the parent chain to
    /// determine the shortest path.
    /// @param _r    The roadmap.
    /// @param _vid  The vertex.
    /// @return The distance of _vid to its root parent.
    double ShortestPathWeight(const RoadmapType* const _r,
        const VID _vid) const noexcept;

    /// @overload
    double ShortestPathWeight(const GroupRoadmapType* const _r,
        const VID _vid) const noexcept;

    /// Determine the edge weight for an existing roadmap edge.
    /// @param _r      The roadmap.
    /// @param _source The source roadmap.
    /// @param _target The target roadmap.
    /// @return The edge weight.
    double EdgeWeight(const RoadmapType* const _r, const VID _source,
        const VID _target) const noexcept;

    /// @overload
    double EdgeWeight(const GroupRoadmapType* const _r, const VID _source,
        const VID _target) const noexcept;

    /// Determine the edge weight for an existing edge.
    /// @param _w The weight object.
    /// @param _robot The robot object.
    /// @return The edge weight.
    double EdgeWeight(const WeightType& _w, const Robot* const _robot) const
        noexcept;

    /// Change the parent of a vertex in a roadmap.
    /// @param _r         The roadmap.
    /// @param _vid       The vertex to rewire.
    /// @param _oldParent The vertex's old parent.
    /// @param _newParent The vertex's new parent.
    /// @param _newLp     The local plan from new parent to the vertex.
    /// @param _newCost   The total cost of the vertex from the root, through
    ///                   the new parent.
    void ChangeParent(RoadmapType* const _r, const VID _vid,
        const VID _oldParent, const VID _newParent, const WeightType& _newLp,
        const double _newCost) const noexcept;

    /// @overload
    void ChangeParent(GroupRoadmapType* const _r, const VID _vid,
        const VID _oldParent, const VID _newParent, const GroupWeightType& _newLp,
        const double _newCost) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    std::string m_nfLabel;        ///< NF for locating rewire attempts.
    std::string m_operatorLabel;  ///< The operator for combining edge weights.
    std::string m_objectiveLabel; ///< The objective function.

    /// Operator for combining edge weights.
    std::function<double(const double&, const double&)> m_operator;
    /// Operator for determining best cost. Returns true iff the second argument
    /// is a better cost than the first.
    std::function<double(const double&, const double&)> m_objective;

    /// Buffer for nf-reported distances.
    std::unordered_map<VID, double> m_distanceBuffer;

    ///@}

};

#endif

