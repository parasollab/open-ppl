#include "RewireConnector.h"

#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"


/*------------------------------- Construction -------------------------------*/

RewireConnector::
RewireConnector() {
  this->SetName("RewireConnector");
  this->m_rewiring = true;
}


RewireConnector::
RewireConnector(XMLNode& _node) : ConnectorMethod(_node) {
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

void
RewireConnector::
ConnectImpl(RoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<RoadmapType>* const _collision) {
  // Determine nearest neighbors.
  const auto& cfg = _r->GetVertex(_source);
  auto nf = this->GetMPLibrary()->GetNeighborhoodFinder(m_nfLabel);
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


void
RewireConnector::
ConnectImpl(GroupRoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<GroupRoadmapType>* const _collision) {
  // Determine nearest neighbors.

  const auto& gcfg = _r->GetVertex(_source);
  auto nf = this->GetMPLibrary()->GetNeighborhoodFinder(m_nfLabel);
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

void
RewireConnector::
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


void
RewireConnector::
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


void
RewireConnector::
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


void
RewireConnector::
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


typename RewireConnector::RewireTestOutput
RewireConnector::
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
  auto nf = this->GetMPLibrary()->GetNeighborhoodFinder(m_nfLabel);
  auto dm = this->GetMPLibrary()->GetDistanceMetric(nf->GetDMLabel());
  const Cfg& potentialParentCfg = _r->GetVertex(_potentialParent),
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
  auto lp    = this->GetMPLibrary()->GetLocalPlanner(this->m_lpLabel);
  auto env   = this->GetEnvironment();
  auto robot = _r->GetRobot();
  Cfg collision(robot);
  LPOutput lpo;

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


typename RewireConnector::GroupRewireTestOutput
RewireConnector::
RewireTest(GroupRoadmapType* const _r, const VID _vid,
    const VID _currentParent,   const double _currentCost,
    const VID _potentialParent, const double _potentialParentCost,
    OutputIterator<GroupRoadmapType>* const _collision) noexcept {

  GroupLPOutput glpo(_r);
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
  auto nf = this->GetMPLibrary()->GetNeighborhoodFinder(m_nfLabel);
  auto dm = this->GetMPLibrary()->GetDistanceMetric(nf->GetDMLabel());
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
  auto lp    = this->GetMPLibrary()->GetLocalPlanner(this->m_lpLabel);
  auto env   = this->GetEnvironment();
  GroupCfgType collision;
  GroupLPOutput lpo(cfg.GetGroupRoadmap());

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


double
RewireConnector::
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


double
RewireConnector::
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


double
RewireConnector::
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


double
RewireConnector::
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


double
RewireConnector::
EdgeWeight(const WeightType& _w, const Robot* const _robot) const noexcept {
  /// @note This will always be the GetWeight version unless we develop a
  ///       steering function and local planner that can use it for a
  ///       nonholonomic robot. I've left it in to remind us that nonholonomic
  ///       robots use time steps for their default cost function.
  return _robot->IsNonholonomic() ? _w.GetTimeSteps()
                                  : _w.GetWeight();
}


void
RewireConnector::
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


void
RewireConnector::
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
