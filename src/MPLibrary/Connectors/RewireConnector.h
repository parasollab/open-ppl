#ifndef PMPL_REWIRE_CONNECTOR_H_
#define PMPL_REWIRE_CONNECTOR_H_

#include "ConnectorMethod.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"

////////////////////////////////////////////////////////////////////////////////
/// Re-wires a tree for optimal RRT planners. This only makes sense for
/// tree-like planners and will break any that track information about
/// the paths or tree structure.
///
/// Reference:
///   Sertac Karaman and Emilio Frazzoli. "Sampling-based algorithms for optimal
///   motion planning". IJRR 2011.
///
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits>
class RewireConnector : public ConnectorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::WeightType        WeightType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename RoadmapType::VID            VID;

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

    ///@}
    ///@name Construction
    ///@{

    RewireConnector();

    RewireConnector(XMLNode& _node);

    virtual ~RewireConnector() = default;

    ///@}
    ///@name Connector Interface
    ///@{

    template <typename InputIterator1, typename InputIterator2,
              typename OutputIterator>
    void Connect(RoadmapType* _r,
        InputIterator1 _itr1First, InputIterator1 _itr1Last,
        InputIterator2 _itr2First, InputIterator2 _itr2Last,
        bool _fromFullRoadmap,
        OutputIterator _collision);

    template <typename InputIterator1, typename InputIterator2,
              typename OutputIterator>
    void Connect(GroupRoadmapType* _r,
        InputIterator1 _itr1First, InputIterator1 _itr1Last,
        InputIterator2 _itr2First, InputIterator2 _itr2Last,
        bool _fromFullRoadmap,
        OutputIterator _collision);

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Attempt to rewire a vertex through its nearest neighbors if doing so
    /// results in a shorter path to the nearest root vertex.
    /// @param _r         The containing roadmap.
    /// @param _vid       The vertex whos shortest path may be re-routed.
    /// @param _neighbors The set of potential new parents for _vid.
    /// @param _collision Output iterator for collisions detected during
    ///                   connection attempts.
    template <typename OutputIterator>
    void RewireVertex(RoadmapType* const _r, const VID _vid,
        const std::vector<Neighbor>& _neighbors, OutputIterator _collision);

    /// Attempt to rewire the nearest-neighbors of a vertex _vid through itself
    /// if doing so results in a shorter path to the nearest root vertex.
    /// @param _r         The containing roadmap.
    /// @param _vid       The vertex whos shortest path may be re-routed.
    /// @param _neighbors The set of potential new parents for _vid.
    /// @param _collision Output iterator for collisions detected during
    ///                   connection attempts.
    template <typename OutputIterator>
    void RewireNeighbors(RoadmapType* const _r, const VID _vid,
        const std::vector<Neighbor>& _neighbors, OutputIterator _collision);

    /// Check if a vertex should be rewired through a new parent.
    /// @param _r                   The containing roadmap.
    /// @param _vid                 The vertex which may be rewired.
    /// @param _currentParent       The vertex's current parent.
    /// @param _currentCost         The vertex's current cost.
    /// @param _potentialParent     The potential parent to check.
    /// @param _potentialParentCost The cost to reach the potential parent.
    /// @param _collision           Output iterator for collisions detected
    ///                             during connection attempts.
    /// @return A rewire test output indicating success and generated lp/cost.
    template <typename OutputIterator>
    RewireTestOutput RewireTest(RoadmapType* const _r, const VID _vid,
        const VID _currentParent, const double _currentCost,
        const VID _potentialParent, const double _potentialParentCost,
        OutputIterator _collision) noexcept;

    /// Trace the path from one vertex to another using the "Parent" stats to
    /// determine the shortest path.
    /// @param _r    The roadmap.
    /// @param _vid  The vertex.
    /// @return The distance of _vid to its root parent.
    double ShortestPathWeight(const RoadmapType* const _r,
        const VID _vid) const noexcept;

    /// Determine the edge weight for an existing roadmap edge.
    /// @param _r      The roadmap.
    /// @param _source The source roadmap.
    /// @param _target The target roadmap.
    /// @return The edge weight.
    double EdgeWeight(const RoadmapType* const _r, const VID _source,
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

/*--------------------------- Connector Interface ----------------------------*/

template <typename MPTraits>
template <typename InputIterator1, typename InputIterator2,
          typename OutputIterator>
void
RewireConnector<MPTraits>::
Connect(RoadmapType* _r,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {
  if(this->m_debug)
    std::cout << this->GetName() << "::Connect"
              << std::endl;

  auto nf = this->GetNeighborhoodFinder(m_nfLabel);

  // Attempt to connect each element in the first range to each element in the
  // second.
  std::vector<Neighbor> closest;
  for(InputIterator1 itr1 = _itr1First; itr1 != _itr1Last; ++itr1) {
    // Get the VID and cfg.
    const VID vid = _r->GetVID(itr1);
    const auto& cfg = _r->GetVertex(vid);

    // Determine nearest neighbors.
    closest.clear();
    nf->FindNeighbors(_r, _itr2First, _itr2Last, _fromFullRoadmap, cfg,
        std::back_inserter(closest));

    // Attempt to rewire this vertex with a better parent.
    RewireVertex(_r, vid, closest, _collision);
    // Attempt to rewire the closest through vid.
    RewireNeighbors(_r, vid, closest, _collision);
  }
}


template <typename MPTraits>
template <typename InputIterator1, typename InputIterator2,
          typename OutputIterator>
void
RewireConnector<MPTraits>::
Connect(GroupRoadmapType* _r,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {
  throw NotImplementedException(WHERE);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
template <typename OutputIterator>
void
RewireConnector<MPTraits>::
RewireVertex(RoadmapType* const _r, const VID _vid,
    const std::vector<Neighbor>& _neighbors, OutputIterator _collision) {
  // Check the current best path cost from the root to _vid.
  double    bestCost      = ShortestPathWeight(_r, _vid);
  auto&     cfg           = _r->GetVertex(_vid);
  const VID oldParentVID  = cfg.GetStat("Parent");
  VID       bestParentVID = oldParentVID;
  LPOutput<MPTraits> bestLP;

  if(this->m_debug)
    std::cout << "\tAttempting to rewire node " << _vid
              << " with cost " << bestCost << " through neighbors"
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
    const double neighborCost = ShortestPathWeight(_r, neighborVID);
    const RewireTestOutput test = RewireTest(_r, _vid, bestParentVID, bestCost,
        neighborVID, neighborCost, _collision);

    // Move on if the test failed.
    if(!test.passed)
      continue;

    // This is possibly the new best.
    bestParentVID = neighborVID;
    bestCost      = test.cost;
    bestLP        = test.lpo;
  }


  // If the best parent hasn't changed, do nothing.
  if(bestParentVID == oldParentVID)
    return;

  // The best parent has changed. Update the roadmap.
  ChangeParent(_r, _vid, oldParentVID, bestParentVID, bestLP.m_edge.first,
      bestCost);
}


template <typename MPTraits>
template <typename OutputIterator>
void
RewireConnector<MPTraits>::
RewireNeighbors(RoadmapType* const _r, const VID _vid,
    const std::vector<Neighbor>& _neighbors, OutputIterator _collision) {
  const double vidCost = ShortestPathWeight(_r, _vid);

  if(this->m_debug)
    std::cout << "\tAttempting to rewire nearby nodes through " << _vid
              << " with cost " << vidCost
              << " for " << m_objectiveLabel << " cost."
              << std::endl;

  // Check if any of the neighbors can get better paths through this node.
  for(const auto& neighbor : _neighbors) {
    // Skip neighbors which have no parent as they are roots.
    const VID neighborVID = neighbor.target;
    auto& neighborCfg = _r->GetVertex(neighborVID);
    if(!neighborCfg.IsStat("Parent")) {
      if(this->m_debug)
        std::cout << "\t\tSkipping node " << neighborVID << " which is a root."
                  << std::endl;
      continue;
    }

    // Test to see if this node should be rewired through _vid.
    const VID neighborCurrentParent = neighborCfg.GetStat("Parent");
    const double currentCost = ShortestPathWeight(_r, neighborVID);
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
template <typename OutputIterator>
typename RewireConnector<MPTraits>::RewireTestOutput
RewireConnector<MPTraits>::
RewireTest(RoadmapType* const _r, const VID _vid,
    const VID _currentParent,   const double _currentCost,
    const VID _potentialParent, const double _potentialParentCost,
    OutputIterator _collision) noexcept {
  // Skip rewiring through the same parent.
  if(_potentialParent == _currentParent) {
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid
                << " which is already a child of " << _potentialParent << "."
                << std::endl;
    return RewireTestOutput();
  }

  // Skip rewiring if the current path to _vid is better than the path to
  // _potential parent.
  if(m_objective(_potentialParentCost, _currentCost)) {
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid << " because current cost "
                << _currentCost << " is better than cost to " << _potentialParent
                << " of " << _potentialParentCost << "."
                << std::endl;
    return RewireTestOutput();
  }

  // Check for previous failures to generate this local plan.
  const bool previouslyFailed = this->IsCached(_potentialParent, _vid);
  if(previouslyFailed) {
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid << " because the path ("
                << _potentialParent << ", " << _vid
                << ") was already found invalid."
                << std::endl;
    return RewireTestOutput();
  }

  // Create a local plan. If we fail, do not rewire.
  auto lp    = this->GetLocalPlanner(this->m_lpLabel);
  auto env   = this->GetEnvironment();
  auto robot = _r->GetRobot();
  CfgType collision(robot);
  LPOutput<MPTraits> lpo;

  const bool success = lp->IsConnected(_r->GetVertex(_potentialParent),
      _r->GetVertex(_vid), collision, &lpo, env->GetPositionRes(),
      env->GetOrientationRes());

  if(!success) {
    _collision++ = collision;
    // Cache both directions since LPs must generate symmetric plans.
    this->CacheFailedConnection(_potentialParent, _vid);
    this->CacheFailedConnection(_vid, _potentialParent);
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid << " because the path ("
                << _potentialParent << ", " << _vid
                << ") is invalid."
                << std::endl;
    return RewireTestOutput();
  }

  // If the total cost isn't better, do not rewire.
  const double potentialCost = _potentialParentCost
                             + EdgeWeight(lpo.m_edge.first, robot);
  if(m_objective(potentialCost, _currentCost)) {
    if(this->m_debug)
      std::cout << "\t\tNot rewiring node " << _vid << " because current cost "
                << _currentCost << " is better than cost through "
                << _potentialParent << " of " << potentialCost << "."
                << std::endl;
    return RewireTestOutput();
  }

  // The cost through the potential parent is better.
  if(this->m_debug)
    std::cout << "\t\tNode " << _vid << " has a better parent "
              << _potentialParent << " yielding path cost " << potentialCost
              << "."
              << std::endl;

  return RewireTestOutput(true, std::move(lpo), potentialCost);
}


template <typename MPTraits>
double
RewireConnector<MPTraits>::
ShortestPathWeight(const RoadmapType* const _r, const VID _vid)
    const noexcept {
  // Backtrack the path from _vid to its root by following the "Parent"
  // stat which is populated by RRT methods.
  VID currentVID = _vid;
  double pathWeight = 0;
  while(true) {
    // Fetch the this cfg and find its parent. If it doesn't have one, it's a
    // root.
    const auto& cfg = _r->GetVertex(currentVID);
    if(!cfg.IsStat("Parent"))
      break;
    const VID parentVID = cfg.GetStat("Parent");

    // Include the weight of edge (parentVID, currentVID) in the path weight.
    const double edgeWeight = EdgeWeight(_r, parentVID, currentVID);
    pathWeight = m_operator(pathWeight, edgeWeight);

    // Move on to the parent vertex.
    currentVID = parentVID;
  }

  return pathWeight;
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
EdgeWeight(const WeightType& _w, const Robot* const _robot) const noexcept {
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
  _r->GetVertex(_vid).SetStat("Parent", _newParent);
  if(this->m_debug)
    std::cout << "\t\tRewiring node " << _vid << " through " << _newParent
              << " at new cost " << _newCost << "."
              << std::endl;
}

/*----------------------------------------------------------------------------*/

#endif
