#ifndef PMPL_TOPOLOGICAL_FILTER_H_
#define PMPL_TOPOLOGICAL_FILTER_H_

#include "NeighborhoodFinderMethod.h"

#include "MPLibrary/DistanceMetrics/TopologicalDistance.h"

#include <string>
#include <unordered_map>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// Filters the roadmap for a set of topologically relevant candidate neighbors.
/// Choosing from amongst the candidates is then delegated to another
/// neighborhood finder.
///
/// Reference:
///   Read Sandstrom, Andrew Bregger, Ben Smith, Shawna Thomas, and Nancy M.
///   Amato. "Topological Nearest-Neighbor Filtering for Sampling-based
///   Planners". ICRA 2018.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TopologicalFilter : public NeighborhoodFinderMethod<MPTraits> {

  ///@name Internal Types
  ///@{

  /// A map from region to SSSP data, which describes the sampling frontier
  /// about the key region.
  typedef std::unordered_map<const WorkspaceRegion*,
      SSSPOutput<WorkspaceDecomposition>> SSSPCache;

  /// An iterator to the SSSP order of discovery (ordered by distance,
  /// ascending).
  typedef typename SSSPOutput<WorkspaceDecomposition>::Ordering::const_iterator
      OrderingMarker;

  /// A pair of markers to track the begin and end of a body's population within
  /// an SSSP ordering.
  typedef std::pair<OrderingMarker, OrderingMarker> PopulationMarkers;

  ///@}

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType                CfgType;
    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename RoadmapType::VID                 VID;
    typedef typename RoadmapType::VertexSet           VertexSet;
    typedef WorkspaceDecomposition::vertex_descriptor VD;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;
    using typename NeighborhoodFinderMethod<MPTraits>::OutputIterator;

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
    ///@name NeighborhoodFinder Overrides
    ///@{

    virtual void FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out) override;

    virtual void FindNeighbors(GroupRoadmapType* const _r,
        const GroupCfgType& _cfg, const VertexSet& _candidates,
        OutputIterator _out) override;

    ///@}

  private:

    ///@name Candidate Neighbors
    ///@{

    /// Find the candidate regions for a given configuration and body.
    /// @param _cfg The configuration we wish to connect to.
    /// @param _bodyIndex The body to use.
    /// @param _inputCandidates The set of allowed candidates, or the roadmap if
    ///                         empty.
    /// @return The set of regions that hold candidate neighbors for _bodyIndex
    ///         at _cfg.
    PopulationMarkers FindCandidateRegions(RoadmapType* const _r,
        const CfgType& _cfg, const size_t _bodyIndex,
        const VertexSet& _inputCandidates);

    /// Find the topological candidate vertices for a given configuration.
    /// @param _query The configuration we wish to connect to.
    /// @param _inputCandidates The set of allowed candidates, or the roadmap if
    ///                         empty.
    /// @return The set of VIDs that are good topological candidates for _query.
    VertexSet FindCandidates(RoadmapType* const _r, const CfgType& _cfg,
        const VertexSet& _inputCandidates = {});

    ///@}
    ///@name Helpers
    ///@{

    /// Lazy-initialize this object on first use.
    void LazyInitialize();

    /// Initialize the query relevance map.
    void BuildQueryMap();

    /// Get the SSSP data for a region, computing it if necessary.
    /// @param _region The region.
    /// @return The SSSP data for _region.
    SSSPOutput<WorkspaceDecomposition>& GetSSSPData(
        const WorkspaceRegion* _region);

    ///@}
    ///@name Internal State
    ///@{

    bool m_initialized{false}; ///< Flag for lazy initialization.

    std::string m_nfLabel; ///< The underlying neighborhood finder label.
    std::string m_tmLabel; ///< The topological map label.

    SSSPCache m_ssspCache; ///< Cache for connectivity data.

    /// Query-relevant adjacency map.
    typename TopologicalMap<MPTraits>::AdjacencyMap m_queryMap;

    double m_backtrackDistance{0}; ///< The distance to back track along cells.

    bool m_fallback{false}; ///< Fall back to the underlying NF on fail?

    bool m_useQueryMap{false}; ///< Use a query-relevant adjacency map?

    /// Try to find the nearest neighborhood for queries in obstacle space?
    bool m_recoverObstSamples{false};

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
TopologicalFilter<MPTraits>::
TopologicalFilter() : NeighborhoodFinderMethod<MPTraits>(Type::OTHER) {
  this->SetName("TopologicalFilter");
}


template <typename MPTraits>
TopologicalFilter<MPTraits>::
TopologicalFilter(XMLNode& _node)
    : NeighborhoodFinderMethod<MPTraits>(_node, Type::OTHER, false) {
  this->SetName("TopologicalFilter");

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

  m_recoverObstSamples = _node.Read("recoverObst", false, m_recoverObstSamples,
      "Try to find the nearest neighborhood for queries in obstacle space?");
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

/*----------------------- NeighborhoodFinder Overrides -----------------------*/

template <typename MPTraits>
void
TopologicalFilter<MPTraits>::
FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  auto stats = this->GetStatClass();
  const std::string id = this->GetNameAndLabel();
  MethodTimer mt(stats, id + "::FindNeighbors");
  stats->IncStat(id + "::NumQueries");

  if(!m_initialized)
    LazyInitialize();

  // Track the average input size.
  const size_t inputSize = _candidates.size();
  stats->GetAverage(id + "::InputSize") += inputSize;

  if(this->m_debug)
    std::cout << id << "::FindNeighbors\n";

  auto nf = this->GetNeighborhoodFinder(m_nfLabel);

  // Find the topological candidate vertices.
  const bool fullRoadmap = _candidates.size() == _r->Size();
  VertexSet topologicalCandidates = FindCandidates(_r, _cfg,
      fullRoadmap ? VertexSet() : _candidates);
  stats->GetAverage(id + "::TopologicalCandidates") +=
      topologicalCandidates.size();

  // If we found no candidates, report fail or fall back to underlying NF.
  if(topologicalCandidates.empty()) {
    stats->IncStat(id + "::NoTopologicalCandidates");

    if(this->m_debug)
      std::cout << "\tNo vertices found in candidate cells."
                << std::endl;

    // Fall back to underlying NF if that option is selected.
    if(m_fallback) {
      if(this->m_debug)
        std::cout << "\tFalling back to underlying nf '" << m_nfLabel << "'."
                  << std::endl;

      nf->FindNeighbors(_r, _cfg, _candidates, _out);
    }
    return;
  }

  // Track information on average candidate set size.
  stats->GetAverage(id + "::UsedCandidates") += topologicalCandidates.size();

  if(this->m_debug)
    std::cout << "Used candidate set."
              << "\n\t|Input Vertices|: " << inputSize
              << "\n\t|Candidates Vertices|: " << topologicalCandidates.size()
              << std::endl;

  // Call the underlying NF on the reduced candidate set.
  nf->FindNeighbors(_r, _cfg, topologicalCandidates, _out);
}


template <typename MPTraits>
void
TopologicalFilter<MPTraits>::
FindNeighbors(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  /// @todo This should be feasible for groups in a similar fashion to
  ///       multibodies.
  throw NotImplementedException(WHERE);
}

/*---------------------------- Candidate Neighbors ---------------------------*/

template <typename MPTraits>
typename TopologicalFilter<MPTraits>::PopulationMarkers
TopologicalFilter<MPTraits>::
FindCandidateRegions(RoadmapType* const _r, const CfgType& _cfg,
    const size_t _bodyIndex, const VertexSet& _inputCandidates) {
  auto stats = this->GetStatClass();
  const std::string id = this->GetNameAndLabel();
  MethodTimer mt(stats, id + "::FindCandidateRegions");

  if(this->m_debug)
    std::cout << "\tLocating regions for body " << _bodyIndex
              << " at cfg " << _cfg.PrettyPrint()
              << std::endl;

  // Find the cell that _cfg lives in.
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto decomposition = tm->GetDecomposition();
  const WorkspaceRegion* rootRegion = tm->LocateRegion(_cfg, _bodyIndex);

  // Check for invalid region. If so, _cfg is in obstacle space. We *can* locate
  // the nearest region, but this does not really make sense with the multibody
  // version since it may produce a set of 'root regions' which are not feasible
  // for the robot.
  if(!rootRegion) {
    if(m_recoverObstSamples)
      rootRegion = tm->LocateNearestRegion(_cfg, _bodyIndex);
    if(!rootRegion) {
      stats->IncStat(id + "::NoRegion");

      if(this->m_debug)
        std::cout << "\t\tRegion not found, sample is in obstacle space."
                  << std::endl;

      return {};
    }
  }

  // Get the SSSP data for this cell.
  const auto& ssspCache = GetSSSPData(rootRegion);
  const auto& ordering = ssspCache.ordering;
  const auto& distance = ssspCache.distance;

  if(this->m_debug)
    std::cout << "\t\tFinding populated cells for body " << _bodyIndex
              << " from region " << *ordering.begin()
              << std::endl;

  // Find the first marker by scanning the distance map for the closest occupied
  // cell.
  PopulationMarkers markers{ordering.begin(), ordering.end()};
  for(; markers.first != ordering.end(); ++markers.first) {
    // Get the next cell.
    auto& region = decomposition->GetRegion(*markers.first);

    // If it isn't populated, keep going.
    if(!tm->IsPopulated(_r, &region, _bodyIndex))
      continue;

    // If the input candidate set is empty, we're looking over the whole
    // roadmap and this is the first marker.
    if(_inputCandidates.empty())
      break;

    // If there are any points in common between this region's VIDs and
    // our input candidate set, we've found the first marker.
    const VertexSet& cellVIDs = tm->GetMappedVIDs(_r, &region, _bodyIndex);
    const VertexSet* small, * large;
    if(cellVIDs.size() < _inputCandidates.size()) {
      small = &cellVIDs;
      large = &_inputCandidates;
    }
    else {
      small = &_inputCandidates;
      large = &cellVIDs;
    }
    const bool sharedElement = std::any_of(small->begin(), small->end(),
        [&large](const VID _v){return large->count(_v);});
    if(sharedElement)
      break;
  }

  // Check for no markers. We may find none if the underlying NF is a radius
  // type, or if the query occurs in a region of workspace that is disconnected
  // from the rest of the roadmap.
  if(markers.first == markers.second) {
    if(this->m_debug)
      std::cout << "\t\t\tNo populated cells." << std::endl;
    return markers;
  }

  if(this->m_debug)
    std::cout << "\t\t\tFirst populated cell " << *markers.first
              << " in order " << std::distance(ordering.begin(), markers.first)
              << " has distance "
              << std::setprecision(4) << distance.at(*markers.first) << "."
              << std::endl;

  // If the underlying NF is radius, we currently need the whole set (might
  // change if we adjust the SSSP computation).
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  if(nf->GetType() == NeighborhoodFinderMethod<MPTraits>::Type::RADIUS)
    return markers;

  // Compute the max distance between the first and second marker.
  const double maxDistance = distance.at(*markers.first)
                           + m_backtrackDistance;

  if(this->m_debug)
    std::cout << "\t\t\tSearching for new last cell with max distance "
              << std::setprecision(4) << maxDistance << "."
              << std::endl;

  // Binary search from the previous end to find the new end.
  auto first = markers.first;
  auto last = markers.second;
  while(first != last) {
    auto midpoint = first;
    midpoint += (last - first) / 2;

    if(this->m_debug) {
      std::cout << "\t\t\titer: " << *first
                << ", " << std::setprecision(4) << distance.at(*first)
                << "\t\t\tmid:  " << *midpoint
                << ", " << std::setprecision(4) << distance.at(*midpoint)
                << "\t\t\tend:  ";
      if(last == ordering.end())
        std::cout << "end iter";
      else
        std::cout << *last
                  << ", " << std::setprecision(4) << distance.at(*last);
      std::cout << std::endl;
    }

    if(distance.at(*midpoint) > maxDistance)
      last = midpoint;
    else if(distance.at(*midpoint) < maxDistance)
      first = ++midpoint;
    else
      break;
  }

  markers.second = first;

  // Scan forward until the last node exceeds the max distance (there could
  // be several at this distance).
  while(markers.second != ordering.end()
      and distance.at(*markers.second) == maxDistance)
    ++markers.second;

  if(this->m_debug) {
    auto last = markers.second - 1;
    std::cout << "\t\t\tComputed last cell as " << *last
              << " in order " << std::distance(ordering.begin(), last)
              << " / " << ordering.size() - 1
              << " has distance "
              << std::setprecision(4) << distance.at(*last) << ".\n"
              << "\t\tFound " << std::distance(markers.first, markers.second)
              << " regions."
              << std::endl;
  }

  return markers;
}


template <typename MPTraits>
typename MPTraits::RoadmapType::VertexSet
TopologicalFilter<MPTraits>::
FindCandidates(RoadmapType* const _r, const CfgType& _cfg,
    const VertexSet& _inputCandidates) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindCandidates");

  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto mb = this->GetTask()->GetRobot()->GetMultiBody();

  VertexSet candidates;

  // For n bodies, make n vertex sets to track the descriptors which have been
  // seen n + 1 times.
  const size_t bodyCount = mb->GetNumBodies();
  std::vector<VertexSet> countSets(bodyCount);

  // Make a map from descriptor to count. Elements not present have zero count.
  std::unordered_map<VID, size_t> descriptorCounts;

  if(this->m_debug)
    std::cout << "Searching for candidate VIDs..." << std::endl;

  // Find the candidates for each body and count the number of times they
  // appear.
  for(size_t i = 0; i < bodyCount; ++i) {
    // Find the candidate regions for this body.
    const PopulationMarkers regions = FindCandidateRegions(_r, _cfg, i,
        _inputCandidates);

    // Get the sorted candidates for this region.
    {
      MethodTimer mt(this->GetStatClass(),
          this->GetNameAndLabel() + "::FindCandidates::Get");
      candidates.clear();
      candidates = tm->GetMappedVIDs(_r, regions.first, regions.second, i);
    }

    if(this->m_debug)
      std::cout << "\t" << candidates.size()
                << " candidate VIDs found for body " << i << "."
                << std::endl;

    // If we are selecting only from a set of input candidates, discard any new
    // candidates that aren't in the input set.
    if(_inputCandidates.size()) {
      MethodTimer mt(this->GetStatClass(),
          this->GetNameAndLabel() + "::FindCandidates::Filter");
      for(auto iter = candidates.begin(); iter != candidates.end(); ) {
        // If this vertex is allowed, move on.
        if(_inputCandidates.count(*iter))
          ++iter;
        else
          iter = candidates.erase(iter);
      }

      if(this->m_debug)
        std::cout << "\t" << candidates.size()
                  << " candidates found in the input set."
                  << std::endl;
    }

    // Count the new candidates.
    MethodTimer mt(this->GetStatClass(),
        this->GetNameAndLabel() + "::FindCandidates::Count");
    for(const auto vid : candidates) {
      // Check if this vertex has already been seen.
      auto iter = descriptorCounts.find(vid);
      const bool seen = iter != descriptorCounts.end();

      // If the vertex hasn't been seen, add it to the first count set.
      if(!seen) {
        countSets[0].insert(vid);
        descriptorCounts[vid] = 1;
      }
      // If it has been seen, move it to the next highest count set.
      else {
        const size_t oldCount = iter->second,
                     oldIndex = oldCount - 1,
                     newIndex = oldCount;
        countSets[oldIndex].erase(vid);
        countSets[newIndex].insert(vid);
        ++iter->second;
      }
    }
  }

  // We have counted all of the descriptors in each body's frontier. Determine
  // the best set to return for the underlying NF.
  MethodTimer jt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindCandidates::Join");
  candidates.clear();
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  switch(nf->GetType()) {
    // For k-nearest, add candidates from the best to worst count set until we
    // have at least k.
    case NeighborhoodFinderMethod<MPTraits>::Type::K:
      {
        const size_t desiredVertices = nf->GetK();
        for(auto iter = countSets.rbegin();
            iter != countSets.rend() and candidates.size() < desiredVertices;
            ++iter)
          VertexSetUnionInPlace(candidates, *iter);
      }
      break;
    // For radius, include the two best non-empty count sets?
    /// @todo Wtf is this. Justify this heuristic or come up with one we can
    ///       justify.
    case NeighborhoodFinderMethod<MPTraits>::Type::RADIUS:
      {
        size_t count = 0;
        const size_t maxCount = 2;
        for(auto iter = countSets.rbegin();
            iter != countSets.rend() and count < maxCount;
            ++iter) {
          if(iter->empty())
            continue;
          VertexSetUnionInPlace(candidates, *iter);
          ++count;
        }
      }
      break;
    default:
      throw RunTimeException(WHERE) << "Underlying NF is not a supported type.";
  }


  if(this->m_debug)
    std::cout << "\tReturning " << candidates.size()
              << " candidates."
              << std::endl;
  return candidates;
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
void
TopologicalFilter<MPTraits>::
LazyInitialize() {
  m_initialized = true;

  m_ssspCache.clear();

  m_queryMap.clear();
  if(m_useQueryMap)
    BuildQueryMap();
}


template <typename MPTraits>
void
TopologicalFilter<MPTraits>::
BuildQueryMap() {
  const std::string id = this->GetNameAndLabel();
  MethodTimer mt(this->GetStatClass(), id + "::BuildQueryMap");

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
  auto g = this->GetRoadmap();
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
  auto goalRegion = tm->LocateRegion(goal);
  const auto sssp = tm->ComputeFrontier(goalRegion);

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
    std::cout << id << "::BuildQueryMap"
              << "\n\tQuery:"
              << "\n\t\t(" << startVID << ") " << start.PrettyPrint()
              << "\n\t\t(" << goalVID  << ") " << goal.PrettyPrint()
              << "\n\tGoal region: " << goalRegion
              << "\n\tDescriptor:  "
              << decomposition->GetDescriptor(*goalRegion)
              << "\n\tMapped VIDs for base:";
    for(const auto vid : tm->GetMappedVIDs(g, goalRegion))
      std::cout << "  " << vid;
    auto startRegion = tm->LocateRegion(start);
    std::cout << "\n\tStart region: " << startRegion
              << "\n\tDescriptor:  " << decomposition->GetDescriptor(*startRegion)
              << "\n\tMapped VIDs for base:";
    for(const auto vid : tm->GetMappedVIDs(g, startRegion))
      std::cout << "  " << vid;

    std::cout << std::endl;
  }
}


template <typename MPTraits>
SSSPOutput<WorkspaceDecomposition>&
TopologicalFilter<MPTraits>::
GetSSSPData(const WorkspaceRegion* _region) {
  auto iter = m_ssspCache.find(_region);
  const bool cacheHit = iter != m_ssspCache.end();

  this->GetStatClass()->GetAverage(this->GetNameAndLabel() + "::CacheHitRate") +=
      cacheHit;

  if(cacheHit)
    return iter->second;

  // Compute the distance map for this region if it is not cached.
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  if(this->m_debug)
    std::cout << "\t\tSSSP cache for region "
              << tm->GetDecomposition()->GetDescriptor(*_region) << " is cold."
              << std::endl;

  // Do the entire search for now. We will worry about pruning it later.
  // With no early-stop condition, the body index doesn't matter.
  /// @todo This fails to leverage a major advantage of the filter in large
  ///       workspaces, which is that we should only need to search until
  ///       discovering the frontier plust backtrack distance. If the frontier
  ///       is close and the decomposition is large, this wastes a lot of
  ///       effort.
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);

  auto& ssspCache = m_ssspCache[_region];
  ssspCache = nf->GetType() == NeighborhoodFinderMethod<MPTraits>::Type::RADIUS
            ? tm->ComputeFrontierNew(_region, nf->GetRadius(), m_queryMap)
            : tm->ComputeFrontier(_region, nullptr, 0, -1, m_queryMap);

  // Remove the data we will not use.
  ssspCache.parent.clear();
  ssspCache.successors.clear();

  return ssspCache;
}

/*----------------------------------------------------------------------------*/

#endif
