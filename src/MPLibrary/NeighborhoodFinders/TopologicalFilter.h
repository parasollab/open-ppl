#ifndef TOPOLOGICAL_FILTER_H_
#define TOPOLOGICAL_FILTER_H_

#include "NeighborhoodFinderMethod.h"

#include "MPLibrary/DistanceMetrics/TopologicalDistance.h"
#include "MPLibrary/MapEvaluators/RRTQuery.h"


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

    ///@}
    ///@name Construction
    ///@{

    TopologicalFilter();

    TopologicalFilter(XMLNode& _node);

    virtual ~TopologicalFilter() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    /// Compute the SSSP scores over the decomposition starting from the goal
    /// region.
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

    /// We don't 'filter' for this case, we merely supply an alternative distance
    /// metric which first sorts by connected workspace distance.
    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(RoadmapType* _rmp,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    ///@}

  private:

    ///@name Candidate Neighbors
    ///@{

    /// Find the candidate regions for connecting to a given configuration.
    /// @param _cfg The configuration we wish to connect to.
    /// @return The set of regions that hold candidate neighbors for _cfg.
    std::vector<const WorkspaceRegion*> FindCandidateRegions(const CfgType& _cfg);

    /// Compute the intersection of an input set with a set of candidate
    /// neighbors.
    /// @param _first The beginning of the input range.
    /// @param _last  The end of the input range.
    /// @param _candidates The candidate set.
    /// @return A set of VIDs that are present in both the input range and
    ///         candidate set.
    template <typename InputIterator>
    std::vector<VID> ComputeIntersection(InputIterator _first,
        InputIterator _last, const std::vector<VID>& _candidates) const;

    ///@}
    ///@name Helpers
    ///@{

    void LazyInitialize();

    ///@}
    ///@name Internal State
    ///@{

    std::string m_nfLabel; ///< The underlying neighborhood finder label.
    std::string m_tmLabel; ///< The topological map label.

    /// The SSSP data cache.
    std::unordered_map<const WorkspaceRegion*,
        typename TopologicalMap<MPTraits>::SSSPOutput> m_ssspCache;

    const WorkspaceRegion* m_goalRegion{nullptr}; ///< Query goal region.
    typename TopologicalMap<MPTraits>::AdjacencyMap m_queryMap; ///< Query-relevant adjacency map.

    double m_backtrackDistance; ///< The distance to back track along cells.

    bool m_fallback{false}; ///< Fall back to the underlying NF on fail?

    bool m_useQueryMap{false}; ///< Use a query-relevant adjacency map?

    bool m_initialized{false}; ///< Is this object ready to use?

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
      "Use a query-relevant adjacency map instead of the full decomposition edge set?");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
TopologicalFilter<MPTraits>::
Initialize() {
  m_initialized = false;
  m_goalRegion = nullptr;
  m_ssspCache.clear();
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
  LazyInitialize();

  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "TopologicalFilter::FindNeighbors");

  // This object only works on the free space roadmap right now. It could be
  // expanded to handle other maps if we want, but for now we will crash if this
  // isn't the free space.
  if(_rmp != this->GetRoadmap())
    throw RunTimeException(WHERE, "Only works on the free space at this time.");

  if(this->m_debug)
    std::cout << "TopologicalFilter::FindNeighbors\n";

  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);

  // Find the topological candidate vertices.
  std::vector<const WorkspaceRegion*> regions = FindCandidateRegions(_cfg);
  std::vector<VID> topologicalCandidates = tm->GetMappedVIDs(regions);

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
              << "\n\tInput Vertices: " << std::distance(_first, _last)
              << "\n\tCandidates Vertices: " << candidates.size()
              << "\n\tCandidate Regions: " << regions.size()
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
  throw RunTimeException(WHERE, "This implementation is really bad. So bad "
      "in fact, that I threw an exception to let you know how bad it is. Please "
      "don't use this until it is ready.");

  /// @TODO Re-write this algorithm so that we actually filter the input set.

  // We don't 'filter' for this case, we merely supply an alternative distance
  // metric which first sorts by connected workspace distance.

  // Get the distance metric label that the underlying NF is using.
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  const std::string originalLabel = nf->GetDMLabel(),
                    newLabel = this->GetNameAndLabel()+ "::DM::" + originalLabel ;

  // Make a custom DM if needed.
  /// @TODO This will fail if we need to use the same TopologicalFilter with
  ///       various underlying NFs (we will crash on the second try to add the
  ///       DM).
  if(!m_dm) {
    m_dm = std::shared_ptr<DistanceMetricMethod<MPTraits>>(new
        TopologicalDistance<MPTraits>(m_tmLabel, originalLabel));
    this->GetMPLibrary()->AddDistanceMetric(m_dm, newLabel);
  }

  // Run the underlying NF with the helper DM, then restore the original DM.
  nf->SetDMLabel(newLabel);
  nf->FindNeighborPairs(_rmp, _first1, _last1, _first2, _last2, _out);
  nf->SetDMLabel(originalLabel);

  return _out;
}

/*---------------------------- Candidate Neighbors ---------------------------*/

template <typename MPTraits>
std::vector<const WorkspaceRegion*>
TopologicalFilter<MPTraits>::
FindCandidateRegions(const CfgType& _cfg) {
  MethodTimer mt(this->GetStatClass(), "TopologicalFilter::FindCandidateRegions");

  if(this->m_debug)
    std::cout << "Locating regions for cfg at " << _cfg.GetPoint()
              << std::endl;

  // Find the cell that _cfg lives in.
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto decomposition = tm->GetDecomposition();
  const WorkspaceRegion* rootRegion = tm->LocateRegion(_cfg);

  // Check for invalid region. If so, _cfg is in obstacle space. Don't bother
  // trying to locate the nearest - tried that and it's too expensive.
  /// @TODO Figure out a better answer than guessing/failing here.
  if(!rootRegion) {
    if(this->m_debug)
      std::cout << "\tRegion not found, sample is in obstacle space."
                << std::endl;
    return {};
  }

  // Compute the distance map for this region if it is not cached.
  if(!m_ssspCache.count(rootRegion)) {
    if(this->m_debug)
      std::cout << "\tSSSP cache for region "
                << decomposition->GetDescriptor(*rootRegion)
                << " (" << rootRegion << ") is cold."
                << std::endl;

    m_ssspCache[rootRegion] = m_useQueryMap ?
        tm->ComputeSSSP(rootRegion, m_backtrackDistance, m_queryMap)
      : tm->ComputeSSSP(rootRegion, m_backtrackDistance);
  }
  // Scan the distance map to find the closest occupied cell.
  auto& ordering = m_ssspCache[rootRegion].ordering;
  auto beginIter = ordering.begin();
  for(; beginIter != ordering.end(); ++beginIter) {
    auto& region = decomposition->GetRegion(*beginIter);

    if(tm->IsPopulated(&region))
      break;
  }

  // If we've reached the end of the ordering and found no populated cells,
  // there are no candidates.
  if(beginIter == ordering.end())
    return {};

  // Binary search to check if we can prune the distance map.
  const auto& distances = m_ssspCache[rootRegion].distances;
  const double maxDistance = distances.at(*beginIter) + m_backtrackDistance;

  if(this->m_debug)
    std::cout << "\tFirst populated cell " << *beginIter
              << " in order " << std::distance(ordering.begin(), beginIter)
              << " has distance "
              << std::setprecision(4) << distances.at(*beginIter)
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
                << ", " << std::setprecision(4) << distances.at(*iter)
                << "\tmid:  " << *midpoint
                << ", " << std::setprecision(4) << distances.at(*midpoint)
                << "\tend:  ";
      if(endIter == ordering.end())
        std::cout << "end iter";
      else
        std::cout << *endIter
                  << ", " << std::setprecision(4) << distances.at(*endIter);
      std::cout << std::endl;
    }

    if(distances.at(*midpoint) > maxDistance)
      endIter = midpoint;
    else if(distances.at(*midpoint) < maxDistance)
      iter = ++midpoint;
    else
      break;
  }

  // Scan forward until the last node exceeds the max distance.
  while(iter != ordering.end() and distances.at(*iter) == maxDistance)
    ++iter;

  if(this->m_debug) {
    std::cout << "\tPruning " << std::distance(iter, ordering.end())
              << " cells from the distance map.";
    if(iter != ordering.end())
      std::cout << "\tLast retained node has distance "
                << std::setprecision(4) << distances.at(*iter) << ".";
    std::cout << std::endl;
  }

  // Prune nodes beyond iter.
  if(iter != ordering.end())
    ordering.erase(iter + 1, ordering.end());

  // Accumulate candidate region.
  std::vector<const WorkspaceRegion*> candidates;
  for(auto iter = beginIter; iter != ordering.end(); ++iter)
    candidates.emplace_back(&decomposition->GetRegion(*iter));

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

  // The input range could be VIDs or vertex iterators. Ask the RoadmapGraph for
  // VIDs.
  auto g = this->GetRoadmap()->GetGraph();
  std::vector<VID> inputRange;
  for(auto iter = _first; iter != _last; ++iter)
    inputRange.push_back(g->GetVID(iter));

  // Sort and uniqueify the input set.
  std::sort(inputRange.begin(), inputRange.end());
  std::unique(inputRange.begin(), inputRange.end());

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
  if(m_initialized)
    return;
  m_initialized = true;

  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto decomposition = tm->GetDecomposition();

  // Get the goal configuration from the query.
  /// @TODO Add an option to enable or disable query relevance.
  /// @TODO Support this without magic XML values.
  auto query = static_cast<RRTQuery<MPTraits>*>(this->GetMapEvaluator("RRTQuery").
      get());
  const CfgType& start = query->GetQuery()[0];
  const CfgType& goal  = query->GetQuery()[1];

  auto g = this->GetRoadmap()->GetGraph();
  std::cout << "Query: " << query->GetQuery().size()
            << "\n\t(" << g->GetVID(start) << ") " << start.PrettyPrint()
            << "\n\t(" << g->GetVID(goal ) << ") " << goal.PrettyPrint()
            << std::endl;

  // Find the goal region and compute its SSSP map.
  m_goalRegion = tm->LocateRegion(goal);
  m_ssspCache[m_goalRegion] = tm->ComputeSSSP(m_goalRegion);

  if(this->m_debug) {
    std::cout << "TopologicalFilter::LazyInitialize"
              << "\n\tGoal region: " << m_goalRegion
              << "\n\tDescriptor:  "
              << decomposition->GetDescriptor(*m_goalRegion)
              << "\n\tMapped VIDs:";
    for(const auto vid : tm->GetMappedVIDs(m_goalRegion))
      std::cout << "  " << vid;
    auto startRegion = tm->LocateRegion(start);
    std::cout << "\n\tStart region: " << startRegion
              << "\n\tDescriptor:  " << decomposition->GetDescriptor(*startRegion)
              << "\n\tMapped VIDs:";
    for(const auto vid : tm->GetMappedVIDs(startRegion))
      std::cout << "  " << vid;

    std::cout << std::endl;
  }

  // Build the topological sort pseudo-DAG using SSSP scores as the ordering
  // value.
  const auto& sssp = m_ssspCache[m_goalRegion];
  m_queryMap.clear();
  if(m_useQueryMap) {
    MethodTimer mt(this->GetStatClass(), "TopologicalFilter::QueryMap");

    for(auto vi = decomposition->begin(); vi != decomposition->end(); ++vi) {
      for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
        const VD source = ei->source(),
                 target = ei->target();
        // If the target has a higher score, it is a child in the successor
        // pseudo-DAG.
        if(sssp.distances.count(source) and
            sssp.distances.count(target) and
            sssp.distances.at(source) <= sssp.distances.at(target))
          m_queryMap[source].push_back(target);
      }
    }
  }
}

/*----------------------------------------------------------------------------*/

#endif
