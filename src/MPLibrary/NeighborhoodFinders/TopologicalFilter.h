#ifndef PMPL_TOPOLOGICAL_FILTER_H_
#define PMPL_TOPOLOGICAL_FILTER_H_

#include "NeighborhoodFinderMethod.h"

#include "MPLibrary/DistanceMetrics/TopologicalDistance.h"


////////////////////////////////////////////////////////////////////////////////
/// Filters the roadmap for a set of topologically relevant candidate neighbors.
/// Choosing from amongst the candidates is then delegated to another
/// neighborhood finder.
///
/// @WARNING This must receive samples whose reference points are in free space
///          to find any neighbors.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TopologicalFilter : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType                CfgType;
    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename RoadmapType::GraphType           GraphType;
    typedef typename GraphType::VID                   VID;
    typedef WorkspaceDecomposition::vertex_descriptor VD;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;

    ///@}
    ///@name Construction
    ///@{

    TopologicalFilter();

    TopologicalFilter(XMLNode& _node);

    virtual ~TopologicalFilter() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinder Functions
    ///@{

    /// Filter the candidate range. Pass only the topologically relevant
    /// candidates to the underlying NF.
    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(RoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(RoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(GroupRoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(GroupRoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    ///@}

  private:

    ///@name Candidate Neighbors
    ///@{

    /// Find the candidate regions for a given configuration and body.
    /// @param _cfg The configuration we wish to connect to.
    /// @param _bodyIndex The body to use.
    /// @return The set of regions that hold candidate neighbors for _bodyIndex
    ///         at _cfg.
    std::vector<const WorkspaceRegion*> FindCandidateRegions(const CfgType& _cfg,
        const size_t _bodyIndex);

    /// Find the topological candidate vertices for a given configuration.
    /// @param _query The configuration we wish to connect to.
    /// @return The set of VIDs that are good topological candidates for _query.
    std::vector<VID> FindCandidates(const CfgType& _cfg);

    /// Compute the intersection of an input set with a set of candidate
    /// neighbors.
    /// @param _first The beginning of the input range.
    /// @param _last  The end of the input range.
    /// @param _candidates The candidate set, which must be sorted.
    /// @return A set of VIDs that are present in both the input range and
    ///         candidate set.
    template <typename InputIterator>
    std::vector<VID> ComputeIntersection(InputIterator _first,
        InputIterator _last, const std::vector<VID>& _candidates) const;

    ///@}
    ///@name Helpers
    ///@{

    /// Lazy-initialize this object on first use.
    void LazyInitialize();

    /// Initialize the query relevance map.
    void BuildQueryMap();

    ///@}
    ///@name Internal State
    ///@{

    bool m_initialized{false}; ///< Flag for lazy initialization.

    std::string m_nfLabel; ///< The underlying neighborhood finder label.
    std::string m_tmLabel; ///< The topological map label.

    /// The SSSP data cache. There is one for each body.
    std::vector<std::unordered_map<const WorkspaceRegion*,
        SSSPOutput<WorkspaceDecomposition>>> m_ssspCache;

    const WorkspaceRegion* m_goalRegion{nullptr}; ///< Query goal region.
    typename TopologicalMap<MPTraits>::AdjacencyMap m_queryMap; ///< Query-relevant adjacency map.

    double m_backtrackDistance; ///< The distance to back track along cells.

    bool m_fallback{false}; ///< Fall back to the underlying NF on fail?

    bool m_useQueryMap{false}; ///< Use a query-relevant adjacency map?

    /// Helper distance metric for working with PRMs.
    std::shared_ptr<DistanceMetricMethod<MPTraits>> m_dm;

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
TopologicalFilter<MPTraits>::
TopologicalFilter() : NeighborhoodFinderMethod<MPTraits>() {
  this->SetName("TopologicalFilter");
}


template <typename MPTraits>
TopologicalFilter<MPTraits>::
TopologicalFilter(XMLNode& _node)
    : NeighborhoodFinderMethod<MPTraits>(_node, false) {
  this->SetName("TopologicalFilter");

  this->m_nfType = Type::OTHER;

  m_nfLabel = _node.Read("nfLabel", true, "", "Label for the underlying NF.");

  m_tmLabel = _node.Read("tmLabel", true, "", "Label for the topological map.");

  m_backtrackDistance = _node.Read("backtrackDistance", true, 0., 0.,
      std::numeric_limits<double>::max(),
      "Maximum distance to backtrack along the decomposition cells. Use the "
      "most conservative value that reliably finds configurations.");

  m_fallback = _node.Read("fallback", false, m_fallback,
      "Fall back to the underlying NF when no candidates are found. This helps "
      "with getting started, but can also cause a loss of benefit in large "
      "workspaces where it takes a long time to cover the space.");

  m_useQueryMap = _node.Read("queryRelevance", false, m_useQueryMap,
      "Use a query-relevant adjacency map instead of the full decomposition "
      "edge set?");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
TopologicalFilter<MPTraits>::
Initialize() {
  m_initialized = false;
}


template <typename MPTraits>
void
TopologicalFilter<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tnfLabel: " << m_nfLabel
      << "\n\ttmLabel: " << m_tmLabel
      << "\n\tBacktrack distance: " << m_backtrackDistance
      << "\n\tFall back to underlying nf: " << m_fallback
      << "\n\tQuery relevance: " << m_useQueryMap
      << std::endl;
}

/*----------------------- NeighborhoodFinder Functions -----------------------*/

template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
TopologicalFilter<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  if(!m_initialized)
    LazyInitialize();

  auto stats = this->GetStatClass();

  // This object only works on the free space roadmap right now. It could be
  // expanded to handle other maps if we want, but for now we will crash if this
  // isn't the free space.
  if(_rmp != this->GetRoadmap())
    throw RunTimeException(WHERE) << "Only works on the free space at this time.";

  if(this->m_debug)
    std::cout << "TopologicalFilter::FindNeighbors\n";

  auto nf = this->GetNeighborhoodFinder(m_nfLabel);

  // Find the topological candidate vertices.
  std::vector<VID> topologicalCandidates = FindCandidates(_cfg);

  // Find the vertices that are in both the input set and topological
  // candidates.
  std::vector<VID> candidates;
  if(_fromFullRoadmap)
    candidates = std::move(topologicalCandidates);
  else
    candidates = ComputeIntersection(_first, _last, topologicalCandidates);

  // If we found no candidates, report fail or fall back to underlying NF.
  if(candidates.empty()) {
    // Distinguish between the two types of no-candidate scenarios.
    if(topologicalCandidates.empty()) {
      stats->IncStat("TopologicalFilter::NoTopologicalCandidates");

      if(this->m_debug)
        std::cout << "\tNo vertices found in candidate cells."
                  << std::endl;
    }
    else {
      stats->IncStat("TopologicalFilter::NoInputCandidates");

      if(this->m_debug)
        std::cout << "\tFound " << topologicalCandidates.size() << " vertices in "
                  << "candidate cells, but none were in the input range."
                  << std::endl;
    }

    // Fall back to underlying NF if that option is selected.
    if(m_fallback) {
      if(this->m_debug)
        std::cout << "\tFalling back to underlying nf '" + m_nfLabel + "'."
                  << std::endl;

      nf->FindNeighbors(_rmp, _first, _last, _fromFullRoadmap, _cfg, _out);
    }
    return _out;
  }

  // Call the underlying NF on the reduced candidate set.
  nf->FindNeighbors(_rmp, candidates.begin(), candidates.end(),
      candidates.size() == this->GetRoadmap()->GetGraph()->get_num_vertices(),
      _cfg, _out);

  // Track information on average candidate set size.
  const double count = stats->GetStat("TopologicalFilter::UsedCandidates"),
               avg = stats->GetStat("TopologicalFilter::AvgCandidates");

  stats->IncStat("TopologicalFilter::UsedCandidates");
  stats->SetStat("TopologicalFilter::AvgCandidates",
      (avg * count + candidates.size()) / (count + 1));

  if(this->m_debug)
    std::cout << "Used candidate set."
              << "\n\t|Input Vertices|: " << std::distance(_first, _last)
              << "\n\t|Candidates Vertices|: " << candidates.size()
              << std::endl;

  return _out;
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
TopologicalFilter<MPTraits>::
FindNeighborPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  if(!m_initialized)
    LazyInitialize();

  auto g = _rmp->GetGraph();

  // This implementation depends on the 'neighbors' vector NOT reallocating.
  // Reserve enough space to preclude that possibility.
  std::vector<Neighbor> neighbors;
  neighbors.reserve(2 * this->GetK());

  // For each vertex in the first set, find the candidates in the last set.
  for(auto iter = _first1; iter != _last1; ++iter) {
    // Track the last used position for neighbors.
    auto oldEnd = neighbors.end();

    // Find up to m_k neighbors of this vertex (appended to the end of
    // neighbors).
    this->FindNeighbors(_rmp, _first2, _last2, false, g->GetVertex(*iter),
        std::back_inserter(neighbors));

    // Set the source VID for the newly found neighbors.
    const VID vid = g->GetVID(iter);
    for(auto iter = oldEnd; iter < neighbors.end(); ++iter)
      iter->source = vid;

    // We now have two sorted ranges of at most m_k elements each and need to
    // merge them.
    std::inplace_merge(neighbors.begin(), oldEnd, neighbors.end());

    // Erase any extra neighbors exceeding m_k.
    if(neighbors.size() > this->GetK())
      neighbors.erase(neighbors.begin() + this->GetK(), neighbors.end());
  }

  // Write the neighbor pairs to the out iterator.
  std::copy(neighbors.begin(), neighbors.end(), _out);

  return _out;
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
TopologicalFilter<MPTraits>::
FindNeighbors(GroupRoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
TopologicalFilter<MPTraits>::
FindNeighborPairs(GroupRoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*---------------------------- Candidate Neighbors ---------------------------*/

template <typename MPTraits>
std::vector<const WorkspaceRegion*>
TopologicalFilter<MPTraits>::
FindCandidateRegions(const CfgType& _cfg, const size_t _bodyIndex) {
  MethodTimer mt(this->GetStatClass(), "TopologicalFilter::FindCandidateRegions");

  if(this->m_debug)
    std::cout << "Locating regions for body " << _bodyIndex
              << " at cfg " << _cfg.PrettyPrint()
              << std::endl;

  // Find the cell that _cfg lives in.
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto decomposition = tm->GetDecomposition();
  const WorkspaceRegion* rootRegion = tm->LocateRegion(_cfg, _bodyIndex);

  // Check for invalid region. If so, _cfg is in obstacle space. Don't bother
  // trying to locate the nearest - tried that and it's too expensive.
  /// @todo Figure out a better answer than guessing/failing here.
  if(!rootRegion) {
    if(this->m_debug)
      std::cout << "\tRegion not found, sample is in obstacle space."
                << std::endl;
    return {};
  }

  // Compute the distance map for this region if it is not cached.
  auto& ssspCache = m_ssspCache[_bodyIndex];
  if(!ssspCache.count(rootRegion)) {
    if(this->m_debug)
      std::cout << "\tSSSP cache for body " << _bodyIndex << ", region "
                << decomposition->GetDescriptor(*rootRegion)
                << " (" << rootRegion << ") is cold."
                << std::endl;

    ssspCache[rootRegion] = m_useQueryMap ?
        tm->ComputeFrontier(rootRegion, _bodyIndex, m_backtrackDistance, m_queryMap)
      : tm->ComputeFrontier(rootRegion, _bodyIndex, m_backtrackDistance);
  }
  // Scan the distance map to find the closest occupied cell.
  auto& ordering = ssspCache[rootRegion].ordering;
  auto beginIter = ordering.begin();
  for(; beginIter != ordering.end(); ++beginIter) {
    auto& region = decomposition->GetRegion(*beginIter);

    if(tm->IsPopulated(&region, _bodyIndex))
      break;
  }

  // If we've reached the end of the ordering and found no populated cells,
  // there are no candidates.
  if(beginIter == ordering.end())
    return {};

  // Binary search to check if we can prune the distance map.
  /// @todo This is disabled for multibodies, see if we can incorporate it.
  if(m_ssspCache.size() == 1) {
    const auto& distance = ssspCache[rootRegion].distance;
    const double maxDistance = distance.at(*beginIter) + m_backtrackDistance;

    if(this->m_debug)
      std::cout << "\tFirst populated cell " << *beginIter
                << " in order " << std::distance(ordering.begin(), beginIter)
                << " has distance "
                << std::setprecision(4) << distance.at(*beginIter)
                << ", max distance is "
                << std::setprecision(4) << maxDistance
                << ".\n\tSearching for new last cell."
                << std::endl;

    auto endIter = ordering.end();
    auto iter = beginIter;
    while(iter != endIter) {
      auto midpoint = iter;
      midpoint += (endIter - iter) / 2;

      if(this->m_debug) {
        std::cout << "\titer: " << *iter
                  << ", " << std::setprecision(4) << distance.at(*iter)
                  << "\tmid:  " << *midpoint
                  << ", " << std::setprecision(4) << distance.at(*midpoint)
                  << "\tend:  ";
        if(endIter == ordering.end())
          std::cout << "end iter";
        else
          std::cout << *endIter
                    << ", " << std::setprecision(4) << distance.at(*endIter);
        std::cout << std::endl;
      }

      if(distance.at(*midpoint) > maxDistance)
        endIter = midpoint;
      else if(distance.at(*midpoint) < maxDistance)
        iter = ++midpoint;
      else
        break;
    }

    // Scan forward until the last node exceeds the max distance.
    while(iter != ordering.end() and distance.at(*iter) == maxDistance)
      ++iter;

    if(this->m_debug) {
      std::cout << "\tPruning " << std::distance(iter, ordering.end())
                << " cells from the distance map.";
      if(iter != ordering.end())
        std::cout << "\tLast retained node has distance "
                  << std::setprecision(4) << distance.at(*iter) << ".";
      std::cout << std::endl;
    }

    // Prune nodes beyond iter.
    if(iter != ordering.end())
      ordering.erase(iter + 1, ordering.end());
  }

  // Accumulate candidate region.
  std::vector<const WorkspaceRegion*> candidates;
  for(auto iter = beginIter; iter != ordering.end(); ++iter)
    candidates.emplace_back(&decomposition->GetRegion(*iter));

  if(this->m_debug)
    std::cout << "\tFound " << candidates.size() << " regions."
              << std::endl;

  return candidates;
}


template <typename MPTraits>
std::vector<typename MPTraits::RoadmapType::GraphType::VID>
TopologicalFilter<MPTraits>::
FindCandidates(const CfgType& _cfg) {
  MethodTimer mt(this->GetStatClass(), "TopologicalFilter::FindCandidates");

  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto mb = this->GetTask()->GetRobot()->GetMultiBody();

  std::vector<VID> candidates, newCandidates, buffer;

  if(this->m_debug)
    std::cout << "Searching for candidate VIDs..." << std::endl;

  // Find the candidate configurations for each body and intersect them as we go.
  // Start with the EE and work backward.
  /// @todo Currently the order doesn't actually matter because we are not
  ///       yet homogenizing our distance-based search. Add this element to each
  ///       subsequent call to FindCandidateRegions.
  /// @todo Figure out how to support tree-like robots here. The current impl
  ///       will only handle chains, and assumes the EE is the last body.
  bool first = true;
  for(size_t i = mb->GetNumBodies() - 1; i != size_t(-1); --i) {
    // Find the candidate regions for this body.
    const std::vector<const WorkspaceRegion*> regions = FindCandidateRegions(
        _cfg, i);

    // Get the sorted candidates for this region.
    newCandidates.clear();
    newCandidates = tm->GetMappedVIDs(regions, i);

    if(this->m_debug)
      std::cout << newCandidates.size() << " candidate VIDs found for body "
                << i << "."
                << std::endl;

    // For the first body, retain all candidates. For each successive body,
    // intersect with existing candidates.
    if(first) {
      first = false;
      std::swap(candidates, newCandidates);
    }
    else {
      buffer.clear();
      buffer.reserve(std::max(candidates.size(), newCandidates.size()));
      std::set_intersection(candidates.begin(), candidates.end(),
                            newCandidates.begin(), newCandidates.end(),
                            std::back_inserter(buffer));
      std::swap(buffer, candidates);

      if(this->m_debug)
        std::cout << "\t" << candidates.size()
                  << " in common with previous bodies."
                  << std::endl;
    }

    // If the candidates are empty, there are no viable neighbors.
    if(candidates.empty())
      break;
  }

  return candidates;
}


template <typename MPTraits>
template <typename InputIterator>
std::vector<typename TopologicalFilter<MPTraits>::VID>
TopologicalFilter<MPTraits>::
ComputeIntersection(InputIterator _first, InputIterator _last,
    const std::vector<VID>& _candidates) const {
  if(_candidates.empty())
    return {};

  MethodTimer mt(this->GetStatClass(), "TopologicalFilter::ComputeIntersection");

  auto g = this->GetRoadmap()->GetGraph();

  // The input range could be VIDs or vertex iterators. Ask the RoadmapGraph for
  // VIDs.
  std::vector<VID> inputRange;
  inputRange.reserve(std::distance(_first, _last));
  for(auto iter = _first; iter != _last; ++iter)
    inputRange.push_back(g->GetVID(iter));

  // Sort and uniqueify the input set.
  std::sort(inputRange.begin(), inputRange.end());
  auto iter = std::unique(inputRange.begin(), inputRange.end());
  inputRange.erase(iter, inputRange.end());

  // Compute the intersection with the candidate set.
  std::vector<VID> intersection;
  intersection.reserve(std::max(inputRange.size(), _candidates.size()));

  std::set_intersection(inputRange.begin(), inputRange.end(),
                        _candidates.begin(), _candidates.end(),
                        std::back_inserter(intersection));

  return intersection;
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
void
TopologicalFilter<MPTraits>::
LazyInitialize() {
  m_initialized = true;

  m_goalRegion = nullptr;

  m_ssspCache.clear();
  m_ssspCache.resize(this->GetTask()->GetRobot()->GetMultiBody()->GetNumBodies());

  m_queryMap.clear();
  if(m_useQueryMap)
    BuildQueryMap();
}


template <typename MPTraits>
void
TopologicalFilter<MPTraits>::
BuildQueryMap() {
  MethodTimer mt(this->GetStatClass(), "TopologicalFilter::BuildQueryMap");

  // Only support single-body robots for now (not sure how this would make sense
  // for multibodies).
  auto task = this->GetTask();
  if(task->GetRobot()->GetMultiBody()->GetNumBodies() > 1)
    throw RunTimeException(WHERE) << "Query relevance option is only supported "
                                  << "for single-body robots.";

  // Only support single-goal tasks; this is inherent to the method.
  const auto& goalConstraints = task->GetGoalConstraints();
  if(goalConstraints.size() > 1)
    throw RunTimeException(WHERE) << "Query relevance option is only supported "
                                  << "for single-goal tasks.";

  // Try to prevent non-point tasks by requiring single VIDs for the start and
  // goal.
  auto goalTracker = this->GetGoalTracker();
  const auto& startVIDs = goalTracker->GetStartVIDs();
  const auto& goalVIDs  = goalTracker->GetGoalVIDs(0);
  if(startVIDs.size() != 1)
    throw RunTimeException(WHERE) << "Exactly one start VID is required, but "
                                  << startVIDs.size() << " were found.";
  if(goalVIDs.size() != 1)
    throw RunTimeException(WHERE) << "Exactly one goal VID is required, but "
                                  << goalVIDs.size() << " were found.";

  // Warn users about my laziness in not implementing a Boundary::Volume
  // function to enforce this assumption.
  std::cerr << "Warning: query relevance option assumes that the task "
            << "constraints are single points in c-space, but does not enforce "
            << "this assumption! The algorithm should still work as long as the "
            << "regions are fairly tight (it is based on a coarse map after all)."
            << std::endl;

  // Get the start and goal vertices.
  auto g = this->GetRoadmap()->GetGraph();
  const VID startVID = *startVIDs.begin(),
            goalVID  = *goalVIDs.begin();
  const CfgType& start = g->GetVertex(startVID),
               & goal  = g->GetVertex(goalVID);

  // Build the topological sort pseudo-DAG using SSSP scores as the ordering
  // value.
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto decomposition = tm->GetDecomposition();

  // Find the goal region and compute its SSSP map. Do not cache it because we
  // will want to recompute on the query-relevant adjacency map.
  m_goalRegion = tm->LocateRegion(goal);
  const auto sssp = tm->ComputeFrontier(m_goalRegion);

  // Finally, build the query map from the sssp data.
  m_queryMap.clear();
  for(auto vi = decomposition->begin(); vi != decomposition->end(); ++vi) {
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      const VD source = ei->source(),
               target = ei->target();
      // If the target has a higher score, it is a child in the successor
      // pseudo-DAG.
      if(sssp.distance.count(source) and
          sssp.distance.count(target) and
          sssp.distance.at(source) <= sssp.distance.at(target))
        m_queryMap[source].push_back(target);
    }
  }

  if(this->m_debug) {
    std::cout << "TopologicalFilter::BuildQueryMap"
              << "\n\tQuery:"
              << "\n\t\t(" << startVID << ") " << start.PrettyPrint()
              << "\n\t\t(" << goalVID  << ") " << goal.PrettyPrint()
              << "\n\tGoal region: " << m_goalRegion
              << "\n\tDescriptor:  "
              << decomposition->GetDescriptor(*m_goalRegion)
              << "\n\tMapped VIDs for base:";
    for(const auto vid : tm->GetMappedVIDs(m_goalRegion))
      std::cout << "  " << vid;
    auto startRegion = tm->LocateRegion(start);
    std::cout << "\n\tStart region: " << startRegion
              << "\n\tDescriptor:  " << decomposition->GetDescriptor(*startRegion)
              << "\n\tMapped VIDs for base:";
    for(const auto vid : tm->GetMappedVIDs(startRegion))
      std::cout << "  " << vid;

    std::cout << std::endl;
  }
}

/*----------------------------------------------------------------------------*/

#endif
