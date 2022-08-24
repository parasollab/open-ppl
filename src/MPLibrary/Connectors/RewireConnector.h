#ifndef PMPL_REWIRE_CONNECTOR_H_
#define PMPL_REWIRE_CONNECTOR_H_

#include "ConnectorMethod.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"

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
template<typename MPTraits>
class RewireConnector : virtual public ConnectorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef typename MPTraits::WeightType        WeightType;
    typedef typename MPTraits::GroupWeightType   GroupWeightType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename RoadmapType::VID            VID;
    typedef typename RoadmapType::VertexSet      VertexSet;

    ///@}
    ///@name Local Types
    ///@{

    /// An aggregate for returning the results of a rewiring check.
    struct RewireTestOutput {
      bool               passed{false}; ///< Should we use this rewiring?
      LPOutput<MPTraits> lpo;           ///< The generated local plan.
      double             cost{0};       ///< New total cost to the rewired node.

      RewireTestOutput(const bool _passed = false,
          LPOutput<MPTraits>&& _lpo = LPOutput<MPTraits>(),
          const double _cost = 0) :
          passed(_passed),
          lpo(_lpo),
          cost(_cost)
      { }
    };

    /// An aggregate for returning the results of a group rewiring check.
    struct GroupRewireTestOutput {
      bool               passed{false}; ///< Should we use this rewiring?
      GroupLPOutput<MPTraits> lpo;      ///< The generated local plan.
      double             cost{0};       ///< New total cost to the rewired node.

      GroupRewireTestOutput(const bool _passed = false,
          GroupLPOutput<MPTraits>&& _lpo = GroupLPOutput<MPTraits>(),
          const double _cost = 0) :
          passed(_passed),
          lpo(_lpo),
          cost(_cost)
      { }
    };

    template <typename AbstractRoadmapType>
    using OutputIterator = typename ConnectorMethod<MPTraits>::template
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

    using ConnectorMethod<MPTraits>::m_neighborBuffer;

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

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
RewireConnector<MPTraits>::
RewireConnector() {
  this->SetName("RewireConnector");
  this->m_rewiring = true;
}


template <typename MPTraits>
RewireConnector<MPTraits>::
RewireConnector(XMLNode& _node) : ConnectorMethod<MPTraits>(_node) {
  this->SetName("RewireConnector");
  this->m_rewiring = true;

  m_nfLabel = _node.Read("nfLabel", true, "",
      "The neighborhood finder for identifying connections to attempt.");

  // Parse the edge-combining operator.
  {
    const std::string choices = "Choices are '+' (add), 'min' (minimum), 'max' "
                                "(maximum).";
    m_operatorLabel = _node.Read("operator", false, "+",
        "Operator for combining edge weights. " + choices);
    if(m_operatorLabel == "+")
      m_operator = [](const double& _a, const double& _b) -> double {
        return _a + _b;
      };
    else if(m_operatorLabel == "min")
      m_operator = [](const double& _a, const double& _b) -> double {
        return std::min(_a, _b);
      };
    else if(m_operatorLabel == "max")
      m_operator = [](const double& _a, const double& _b) -> double {
        return std::max(_a, _b);
      };
    else
      throw ParseException(_node.Where()) << "Unrecognized operator '"
                                          << m_operatorLabel << "'. "
                                          << choices
                                          << std::endl;
  }

  // Parse the objective function.
  {
    const std::string choices = "Choices are 'min' (minimum) or 'max' (maximum).";
    m_objectiveLabel = _node.Read("objective", false, "min",
        "Objective function to optimize path length. " + choices);
    if(m_objectiveLabel == "min")
      m_objective = [](const double& _old, const double& _new) -> double {
        return _new < _old;
      };
    else if(m_objectiveLabel == "max")
      m_objective = [](const double& _old, const double& _new) -> double {
        return _new > _old;
      };
    else
      throw ParseException(_node.Where()) << "Unrecognized objective '"
                                          << m_objectiveLabel << "'. "
                                          << choices
                                          << std::endl;
  }
}

/*------------------------ ConnectorMethod Overrides -------------------------*/

template <typename MPTraits>
void
RewireConnector<MPTraits>::
ConnectImpl(RoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<RoadmapType>* const _collision) {
  // Determine nearest neighbors.
  const auto& cfg = _r->GetVertex(_source);
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  m_neighborBuffer.clear();
  if(_targetSet) {
    nf->FindNeighbors(_r, cfg, *_targetSet, std::back_inserter(m_neighborBuffer));
  }
  else {
    nf->FindNeighbors(_r, cfg, std::back_inserter(m_neighborBuffer));
  }

  // Set neighbors' distance to the cost from the tree root, then re-sort
  // according to that cost plus the distance to _source.
  m_distanceBuffer.clear();
  for(auto& neighbor : m_neighborBuffer) {
    m_distanceBuffer[neighbor.target] = neighbor.distance;
    neighbor.distance = ShortestPathWeight(_r, neighbor.target);
  }
  std::sort(m_neighborBuffer.begin(), m_neighborBuffer.end(),
      [this](const Neighbor& _n1, const Neighbor& _n2) {
        return _n1.distance + this->m_distanceBuffer[_n1.target]
             < _n2.distance + this->m_distanceBuffer[_n2.target];
      }
  );

  // If we have a CC tracker, disable it during rewire since we won't change the
  // CCs by the end.
  auto ccTracker = _r->GetCCTracker();
  if(ccTracker)
    ccTracker->Disable();

  // Attempt to rewire this vertex with a better parent.
  RewireVertex(_r, _source, m_neighborBuffer, _collision);
  // Attempt to rewire the neighbors through _source.
  RewireNeighbors(_r, _source, m_neighborBuffer, _collision);

  if(ccTracker)
    ccTracker->Enable();
}

template <typename MPTraits>
void
RewireConnector<MPTraits>::
ConnectImpl(GroupRoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<GroupRoadmapType>* const _collision) {
  // Determine nearest neighbors.

  const auto& gcfg = _r->GetVertex(_source);
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  m_neighborBuffer.clear();
  if(_targetSet) {
    nf->FindNeighbors(_r, gcfg, *_targetSet, std::back_inserter(m_neighborBuffer));
  }
  else {
    nf->FindNeighbors(_r, gcfg, std::back_inserter(m_neighborBuffer));
  }

  // Set neighbors' distance to the cost from the tree root, then re-sort
  // according to that cost plus the distance to _source.
  m_distanceBuffer.clear();
  for(auto& neighbor : m_neighborBuffer) {
    m_distanceBuffer[neighbor.target] = neighbor.distance;
    neighbor.distance = ShortestPathWeight(_r, neighbor.target);
  }
  std::sort(m_neighborBuffer.begin(), m_neighborBuffer.end(),
      [this](const Neighbor& _n1, const Neighbor& _n2) {
        return _n1.distance + this->m_distanceBuffer[_n1.target]
             < _n2.distance + this->m_distanceBuffer[_n2.target];
      }
  );

  // If we have a CC tracker, disable it during rewire since we won't change the
  // CCs by the end.
  auto ccTracker = _r->GetCCTracker();
  if(ccTracker)
    ccTracker->Disable();

  // Attempt to rewire this vertex with a better parent.
  RewireVertex(_r, _source, m_neighborBuffer, _collision);
  // Attempt to rewire the neighbors through _source.
  RewireNeighbors(_r, _source, m_neighborBuffer, _collision);

  if(ccTracker)
    ccTracker->Enable();
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
RewireConnector<MPTraits>::
RewireVertex(RoadmapType* const _r, const VID _vid,
    const std::vector<Neighbor>& _neighbors,
    OutputIterator<RoadmapType>* const _collision) {
  // Check the current best path cost from the root to _vid.
  const double oldCost    = ShortestPathWeight(_r, _vid);
  const VID oldParentVID  = *_r->GetPredecessors(_vid).begin();

  if(this->m_debug)
    std::cout << "\tAttempting to rewire node " << _vid
              << " with cost " << oldCost << " through neighbors"
              << " for " << m_objectiveLabel << " cost."
              << "\n\t\tCurrent parent is " << oldParentVID << "."
              << std::endl;

  // Check for a new best parent for this node.
  for(const auto& neighbor : _neighbors) {
    // Skip the original parent if it appears again.
    const VID neighborVID = neighbor.target;
    if(neighborVID == oldParentVID) {
      if(this->m_debug)
        std::cout << "\t\tSkipping node " << neighborVID << " which was the "
                  << "previous parent."
                  << std::endl;
      continue;
    }

    // Test if this neighbor is a better parent for _vid than the current best.
    const double neighborCost = neighbor.distance;
    const RewireTestOutput test = RewireTest(_r, _vid, oldParentVID, oldCost,
        neighborVID, neighborCost, _collision);

    // Move on if the test failed.
    if(!test.passed)
      continue;

    // This is the new best.
    ChangeParent(_r, _vid, oldParentVID, neighborVID, test.lpo.m_edge.first,
        test.cost);
    break;
  }
}


template <typename MPTraits>
void
RewireConnector<MPTraits>::
RewireVertex(GroupRoadmapType* const _r, const VID _vid,
    const std::vector<Neighbor>& _neighbors,
    OutputIterator<GroupRoadmapType>* const _collision) {
  // Check the current best path cost from the root to _vid.
  const double oldCost    = ShortestPathWeight(_r, _vid);
  const VID oldParentVID  = *_r->GetPredecessors(_vid).begin();

  if(this->m_debug)
    std::cout << "\tAttempting to rewire node " << _vid
              << " with cost " << oldCost << " through neighbors"
              << " for " << m_objectiveLabel << " cost."
              << "\n\t\tCurrent parent is " << oldParentVID << "."
              << std::endl;

  // Check for a new best parent for this node.
  for(const auto& neighbor : _neighbors) {
    // Skip the original parent if it appears again.
    const VID neighborVID = neighbor.target;
    if(neighborVID == oldParentVID) {
      if(this->m_debug)
        std::cout << "\t\tSkipping node " << neighborVID << " which was the "
                  << "previous parent."
                  << std::endl;
      continue;
    }

    // Test if this neighbor is a better parent for _vid than the current best.
    const double neighborCost = neighbor.distance;
    const GroupRewireTestOutput test = RewireTest(_r, _vid, oldParentVID, oldCost,
        neighborVID, neighborCost, _collision);

    // Move on if the test failed.
    if(!test.passed)
      continue;

    // This is the new best.
    ChangeParent(_r, _vid, oldParentVID, neighborVID, test.lpo.m_edge.first,
        test.cost);
    break;
  }
}


template <typename MPTraits>
void
RewireConnector<MPTraits>::
RewireNeighbors(RoadmapType* const _r, const VID _vid,
    const std::vector<Neighbor>& _neighbors,
    OutputIterator<RoadmapType>* const _collision) {
  const double vidCost = ShortestPathWeight(_r, _vid);

  if(this->m_debug)
    std::cout << "\tAttempting to rewire nearby nodes through " << _vid
              << " with cost " << vidCost
              << " for " << m_objectiveLabel << " cost."
              << std::endl;

  // Check if any of the neighbors can get better paths through this node.
  for(const auto& neighbor : _neighbors) {
    // Check for roots and non-tree graphs.
    const VID neighborVID = neighbor.target;
    const auto& predecessors = _r->GetPredecessors(neighborVID);

    switch(predecessors.size()) {
      case 0:
        // This is a root.
        if(this->m_debug)
          std::cout << "\t\tSkipping node " << neighborVID << " which is a root."
                    << std::endl;
        continue;
      case 1:
        // There is one parent. Continue processing.
        break;
      default:
        // The graph is not a tree.
        throw RunTimeException(WHERE) << "Node " << neighborVID << " has "
                                      << predecessors.size() << " parents and is "
                                      << "therefore not a tree. RewireConnector "
                                      << "only works for tree graphs."
                                      << std::endl;
    }

    // Test to see if this node should be rewired through _vid.
    const VID neighborCurrentParent = *predecessors.begin();
    const double currentCost = neighbor.distance;
    const RewireTestOutput test = RewireTest(_r, neighborVID,
        neighborCurrentParent, currentCost, _vid, vidCost, _collision);

    // If the test failed, don't rewire.
    if(!test.passed)
      continue;

    // _vid is a better parent for neighborVID. Rewire the tree.
    ChangeParent(_r, neighborVID, neighborCurrentParent, _vid,
        test.lpo.m_edge.first, test.cost);
  }
}


template <typename MPTraits>
void
RewireConnector<MPTraits>::
RewireNeighbors(GroupRoadmapType* const _r, const VID _vid,
    const std::vector<Neighbor>& _neighbors,
    OutputIterator<GroupRoadmapType>* const _collision) {
  const double vidCost = ShortestPathWeight(_r, _vid);

  if(this->m_debug)
    std::cout << "\tAttempting to rewire nearby nodes through " << _vid
              << " with cost " << vidCost
              << " for " << m_objectiveLabel << " cost."
              << std::endl;

  // Check if any of the neighbors can get better paths through this node.
  for(const auto& neighbor : _neighbors) {
    // Check for roots and non-tree graphs.
    const VID neighborVID = neighbor.target;
    const auto& predecessors = _r->GetPredecessors(neighborVID);

    switch(predecessors.size()) {
      case 0:
        // This is a root.
        if(this->m_debug)
          std::cout << "\t\tSkipping node " << neighborVID << " which is a root."
                    << std::endl;
        continue;
      case 1:
        // There is one parent. Continue processing.
        break;
      default:
        // The graph is not a tree.
        throw RunTimeException(WHERE) << "Node " << neighborVID << " has "
                                      << predecessors.size() << " parents and is "
                                      << "therefore not a tree. RewireConnector "
                                      << "only works for tree graphs."
                                      << std::endl;
    }

    // Test to see if this node should be rewired through _vid.
    const VID neighborCurrentParent = *predecessors.begin();
    const double currentCost = neighbor.distance;
    const GroupRewireTestOutput test = RewireTest(_r, neighborVID,
        neighborCurrentParent, currentCost, _vid, vidCost, _collision);

    // If the test failed, don't rewire.
    if(!test.passed)
      continue;

    // _vid is a better parent for neighborVID. Rewire the tree.
    ChangeParent(_r, neighborVID, neighborCurrentParent, _vid,
        test.lpo.m_edge.first, test.cost);
  }
}


template <typename MPTraits>
typename RewireConnector<MPTraits>::RewireTestOutput
RewireConnector<MPTraits>::
RewireTest(RoadmapType* const _r, const VID _vid,
    const VID _currentParent,   const double _currentCost,
    const VID _potentialParent, const double _potentialParentCost,
    OutputIterator<RoadmapType>* const _collision) noexcept {
  // Skip rewiring through the same parent.
  const bool sameParent = _potentialParent == _currentParent;
  if(sameParent) {
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid
                << " which is already a child of " << _potentialParent << "."
                << std::endl;
    return RewireTestOutput();
  }

  // Check for previous failures to generate this local plan.
  const bool previouslyFailed = this->IsCached(_r, _potentialParent, _vid);
  if(previouslyFailed) {
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid << " because the path ("
                << _potentialParent << ", " << _vid
                << ") was already found invalid."
                << std::endl;
    return RewireTestOutput();
  }

  // Skip rewiring if the current path to _vid is better than the path to
  // _potential parent.
  const bool notBetterTo = m_objective(_potentialParentCost, _currentCost);
  if(notBetterTo) {
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid << " because current cost "
                << _currentCost << " is better than cost to " << _potentialParent
                << " of " << _potentialParentCost << "."
                << std::endl;
    return RewireTestOutput();
  }

  // If the total cost isn't better, do not rewire.
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  auto dm = this->GetDistanceMetric(nf->GetDMLabel());
  const CfgType& potentialParentCfg = _r->GetVertex(_potentialParent),
               & cfg                = _r->GetVertex(_vid);
  const double distance      = dm->Distance(potentialParentCfg, cfg),
               potentialCost = _potentialParentCost + distance;
  const bool notBetterThrough = m_objective(potentialCost, _currentCost);
  if(notBetterThrough) {
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid << " because current cost "
                << _currentCost << " is better than cost through "
                << _potentialParent << " of " << potentialCost << "."
                << std::endl;
    return RewireTestOutput();
  }

  // Create a local plan. If we fail, do not rewire.
  auto lp    = this->GetLocalPlanner(this->m_lpLabel);
  auto env   = this->GetEnvironment();
  auto robot = _r->GetRobot();
  CfgType collision(robot);
  LPOutput<MPTraits> lpo;

  const bool success = lp->IsConnected(potentialParentCfg, cfg, collision, &lpo,
      env->GetPositionRes(), env->GetOrientationRes());

  if(!success) {
    if(_collision)
      *_collision = collision;
    // Cache both directions since LPs must generate symmetric plans.
    this->CacheFailedConnection(_r, _potentialParent, _vid);
    this->CacheFailedConnection(_r, _vid, _potentialParent);
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid << " because the path ("
                << _potentialParent << ", " << _vid
                << ") is invalid."
                << std::endl;
    return RewireTestOutput();
  }

  // Ensure that the computed LP cost is approximately equal to the distance
  // metric cost (or none of this works because we didn't test the right set of
  // neighbors in the first place).
  /// @note You can disable this if your DM doesn't exactly agree with the LP,
  ///       but you will get near-AO instead of AO rewiring since the new parent
  ///       may not be the absolute best one.
#if 1
  const double computedDistance = EdgeWeight(lpo.m_edge.first, robot),
               tolerance        = distance * .001;
  const bool lpDmDisagreement = !nonstd::approx(computedDistance, distance,
      tolerance);
  if(lpDmDisagreement)
    throw RunTimeException(WHERE) << "LP computed distance " << computedDistance
                                  << " differs from DM value " << distance
                                  << ". These must agree for RewireConnector "
                                  << "to function properly.";
#endif

  // The cost through the potential parent is better.
  if(this->m_debug)
    std::cout << "\t\tNode " << _vid << " has a better parent "
              << _potentialParent << " yielding path cost " << potentialCost
              << "."
              << std::endl;

  return RewireTestOutput(true, std::move(lpo), potentialCost);
}


template <typename MPTraits>
typename RewireConnector<MPTraits>::GroupRewireTestOutput
RewireConnector<MPTraits>::
RewireTest(GroupRoadmapType* const _r, const VID _vid,
    const VID _currentParent,   const double _currentCost,
    const VID _potentialParent, const double _potentialParentCost,
    OutputIterator<GroupRoadmapType>* const _collision) noexcept {

  GroupLPOutput<MPTraits> glpo(_r);
  // Skip rewiring through the same parent.
  const bool sameParent = _potentialParent == _currentParent;
  if(sameParent) {
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid
                << " which is already a child of " << _potentialParent << "."
                << std::endl;
    return GroupRewireTestOutput(false, std::move(glpo), 0.0);
  }

  // Check for previous failures to generate this local plan.
  const bool previouslyFailed = this->IsCached(_r, _potentialParent, _vid);
  if(previouslyFailed) {
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid << " because the path ("
                << _potentialParent << ", " << _vid
                << ") was already found invalid."
                << std::endl;
    return GroupRewireTestOutput(false, std::move(glpo), 0.0);
  }

  // Skip rewiring if the current path to _vid is better than the path to
  // _potential parent.
  const bool notBetterTo = m_objective(_potentialParentCost, _currentCost);
  if(notBetterTo) {
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid << " because current cost "
                << _currentCost << " is better than cost to " << _potentialParent
                << " of " << _potentialParentCost << "."
                << std::endl;
    return GroupRewireTestOutput(false, std::move(glpo), 0.0);
  }

  // If the total cost isn't better, do not rewire.
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  auto dm = this->GetDistanceMetric(nf->GetDMLabel());
  const GroupCfgType& potentialParentCfg = _r->GetVertex(_potentialParent),
               & cfg                     = _r->GetVertex(_vid);
  const double distance      = dm->Distance(potentialParentCfg, cfg),
               potentialCost = _potentialParentCost + distance;
  const bool notBetterThrough = m_objective(potentialCost, _currentCost);
  if(notBetterThrough) {
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid << " because current cost "
                << _currentCost << " is better than cost through "
                << _potentialParent << " of " << potentialCost << "."
                << std::endl;
    return GroupRewireTestOutput(false, std::move(glpo), 0.0);
  }

  // Create a local plan. If we fail, do not rewire.
  auto lp    = this->GetLocalPlanner(this->m_lpLabel);
  auto env   = this->GetEnvironment();
  GroupCfgType collision;
  GroupLPOutput<MPTraits> lpo(cfg.GetGroupRoadmap());

  const bool success = lp->IsConnected(potentialParentCfg, cfg, collision, &lpo,
      env->GetPositionRes(), env->GetOrientationRes());

  if(!success) {
    if(_collision)
      *_collision = collision;
    // Cache both directions since LPs must generate symmetric plans.
    this->CacheFailedConnection(_r, _potentialParent, _vid);
    this->CacheFailedConnection(_r, _vid, _potentialParent);
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid << " because the path ("
                << _potentialParent << ", " << _vid
                << ") is invalid."
                << std::endl;
    return GroupRewireTestOutput(false, std::move(glpo), 0.0);
  }

  // Ensure that the computed LP cost is approximately equal to the distance
  // metric cost (or none of this works because we didn't test the right set of
  // neighbors in the first place).
  /// @note You can disable this if your DM doesn't exactly agree with the LP,
  ///       but you will get near-AO instead of AO rewiring since the new parent
  ///       may not be the absolute best one.
#if 1
  const double computedDistance = lpo.m_edge.first.GetWeight(),
               tolerance        = distance * .001;
  const bool lpDmDisagreement = !nonstd::approx(computedDistance, distance,
      tolerance);
  if(lpDmDisagreement)
    throw RunTimeException(WHERE) << "LP computed distance " << computedDistance
                                  << " differs from DM value " << distance
                                  << ". These must agree for RewireConnector "
                                  << "to function properly.";
#endif

  // The cost through the potential parent is better.
  if(this->m_debug)
    std::cout << "\t\tNode " << _vid << " has a better parent "
              << _potentialParent << " yielding path cost " << potentialCost
              << "."
              << std::endl;

  return GroupRewireTestOutput(true, std::move(lpo), potentialCost);
}


template <typename MPTraits>
double
RewireConnector<MPTraits>::
ShortestPathWeight(const RoadmapType* const _r, const VID _vid)
    const noexcept {
  // Backtrack the path from _vid to its root by following the predecessor chain.
  VID currentVID = _vid;
  double pathWeight = 0;

  while(true) {
    // Get the predecessor list and check if we're done.
    const auto& predecessors = _r->GetPredecessors(currentVID);
    switch(predecessors.size()) {
      case 0:
        // This node is a root. We've traced the whole path and can return the
        // weight now.
        return pathWeight;
      case 1:
        // This node has a parent. Continue processing.
        break;
      default:
        // The graph is not a tree.
        throw RunTimeException(WHERE) << "Node " << currentVID << " has "
                                      << predecessors.size() << " parents and "
                                      << "is therefore not a tree. "
                                      << "RewireConnector only works for tree "
                                      << "graphs."
                                      << std::endl;
    }

    // Add the edge weight to our path length and continue up the chain.
    const VID parentVID = *predecessors.begin();

    const double edgeWeight = EdgeWeight(_r, parentVID, currentVID);
    pathWeight = m_operator(pathWeight, edgeWeight);

    currentVID = parentVID;
  }

  throw RunTimeException(WHERE) << "We've arrived at an unreachable state. "
                                << "https://xkcd.com/2200/";
}


template <typename MPTraits>
double
RewireConnector<MPTraits>::
ShortestPathWeight(const GroupRoadmapType* const _r, const VID _vid)
    const noexcept {
  // Backtrack the path from _vid to its root by following the predecessor chain.
  VID currentVID = _vid;
  double pathWeight = 0;

  while(true) {
    // Get the predecessor list and check if we're done.
    const auto& predecessors = _r->GetPredecessors(currentVID);
    switch(predecessors.size()) {
      case 0:
        // This node is a root. We've traced the whole path and can return the
        // weight now.
        return pathWeight;
      case 1:
        // This node has a parent. Continue processing.
        break;
      default:
        // The graph is not a tree.
        throw RunTimeException(WHERE) << "Node " << currentVID << " has "
                                      << predecessors.size() << " parents and "
                                      << "is therefore not a tree. "
                                      << "RewireConnector only works for tree "
                                      << "graphs."
                                      << std::endl;
    }

    // Add the edge weight to our path length and continue up the chain.
    const VID parentVID = *predecessors.begin();

    const double edgeWeight = EdgeWeight(_r, parentVID, currentVID);
    pathWeight = m_operator(pathWeight, edgeWeight);

    currentVID = parentVID;
  }

  throw RunTimeException(WHERE) << "We've arrived at an unreachable state. "
                                << "https://xkcd.com/2200/";
}


template <typename MPTraits>
double
RewireConnector<MPTraits>::
EdgeWeight(const RoadmapType* const _r, const VID _source, const VID _target)
    const noexcept {
  // Ensure the edge exists.
  typename RoadmapType::CEI edge;
  if(!_r->GetEdge(_source, _target, edge))
    throw RunTimeException(WHERE) << "Requested edge (" << _source << ", "
                                  << _target << ") does not exist in roadmap "
                                  << _r << "."
                                  << std::endl;

  return EdgeWeight(edge->property(), _r->GetRobot());
}


template <typename MPTraits>
double
RewireConnector<MPTraits>::
EdgeWeight(const GroupRoadmapType* const _r, const VID _source, const VID _target)
    const noexcept {
  // Ensure the edge exists.
  typename GroupRoadmapType::CEI edge;
  if(!_r->GetEdge(_source, _target, edge))
    throw RunTimeException(WHERE) << "Requested edge (" << _source << ", "
                                  << _target << ") does not exist in roadmap "
                                  << _r << "."
                                  << std::endl;

  /*
  @TODO: the code below was created on a comment from the last review, namely,
         to match the format of this function with the one below it. However,
         there's something a bit odd about it. The type of edge->property() is
         GroupLocalPlan, which only uses a double for edge weight. Therefore,
         it seems that if the map is setup to use timesteps, this will work
         properly, likewise for if it is instead setup to use a distance metric.
         Otherwise, more code is needed there to make something like the below
         work.
  // Determine if we should use time steps (for nonholonomic robots) or weights.
  bool isNonholo = false;
  // Not ideal, but we need to hack our way in to get the group. We won't
  //  mutate it though, just read.
  auto robotGroup = const_cast<GroupRoadmapType*>(_r)->GetGroup();

  for (auto it = robotGroup->begin(); it != robotGroup->end(); ++it) {
    isNonholo = isNonholo || (*it)->IsNonholonomic();
  }

  return isNonholo ? edge->property().GetTimeSteps()
                   : edge->property().GetWeight();
  */
  return edge->property().GetWeight();
}


template <typename MPTraits>
double
RewireConnector<MPTraits>::
EdgeWeight(const WeightType& _w, const Robot* const _robot) const noexcept {
  /// @note This will always be the GetWeight version unless we develop a
  ///       steering function and local planner that can use it for a
  ///       nonholonomic robot. I've left it in to remind us that nonholonomic
  ///       robots use time steps for their default cost function.
  return _robot->IsNonholonomic() ? _w.GetTimeSteps()
                                  : _w.GetWeight();
}


template <typename MPTraits>
void
RewireConnector<MPTraits>::
ChangeParent(RoadmapType* const _r, const VID _vid, const VID _oldParent,
    const VID _newParent, const WeightType& _newLp, const double _newCost)
    const noexcept {
  _r->DeleteEdge(_oldParent, _vid);
  _r->AddEdge(_newParent, _vid, _newLp);
  if(this->m_debug)
    std::cout << "\t\tRewiring node " << _vid << " through " << _newParent
              << " at new cost " << _newCost << "."
              << std::endl;
}


template <typename MPTraits>
void
RewireConnector<MPTraits>::
ChangeParent(GroupRoadmapType* const _r, const VID _vid, const VID _oldParent,
    const VID _newParent, const GroupWeightType& _newLp, const double _newCost)
    const noexcept {
  _r->DeleteEdge(_oldParent, _vid);
  _r->AddEdge(_newParent, _vid, _newLp);
  if(this->m_debug)
    std::cout << "\t\tRewiring node " << _vid << " through " << _newParent
              << " at new cost " << _newCost << "."
              << std::endl;
}

/*----------------------------------------------------------------------------*/

#endif

