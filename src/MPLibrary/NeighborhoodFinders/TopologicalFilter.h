#ifndef TOPOLOGICAL_FILTER_H_
#define TOPOLOGICAL_FILTER_H_

#include "NeighborhoodFinderMethod.h"

#include "MPLibrary/MapEvaluators/RRTQuery.h"


////////////////////////////////////////////////////////////////////////////////
/// Filters the roadmap for a set of topologically relevant candidate neighbors.
/// Choosing from amongst the candidates is then delegated to another
/// neighborhood finder.
///
/// @WARNING This must be used with a free-space sampler. At least the reference
///          point must be in free space for this object to be helpful.
///
/// @TODO If performance is poor, try caching the N most recently used lookups.
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
    typedef WorkspaceDecomposition::edge_descriptor   ED;

    ///@}
    ///@name Local Types
    ///@{

    /// A descriptor-based adjacency map for the successors from an SSSP run.
    typedef std::unordered_map<VD, std::vector<VD>> AdjacencyMap;


    ////////////////////////////////////////////////////////////////////////////
    /// The output of an SSSP run.
    ////////////////////////////////////////////////////////////////////////////
    struct SSSPOutput {

      std::unordered_map<VD, double> distances; ///< Distance to each cell from start.
      std::vector<VD> ordering;                 ///< Cell discovery ordering.
      AdjacencyMap successors;                  ///< Maps predecessor -> successors.

    };

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

    /// For each vertex in one range, call the underlying NF only on those
    /// vertices in the second range which are topological candidates.
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
    ///@name SSSP
    ///@{

    /// Compute the weight map between adjacent decomposition cells.
    void ComputeWeightMap();

    /// Compute the SSSP through a workspace decomposition graph starting from a
    /// designated region.
    /// @param _decomposition The decomposition.
    /// @param _region The starting region.
    /// @param _earlyStop Stop after the last cell added is backtrack distance
    ///                   away from the first populated cell.
    /// @param _adjacency An optional adjacency map to use instead of the
    ///                   decomposition's.
    /// @return The cell distances and successor tree.
    SSSPOutput
    ComputeSSSP(const WorkspaceRegion* const _region,
        const bool _earlyStop = false,
        const AdjacencyMap& _adjacency = {});

    ///@}
    ///@name Internal State
    ///@{

    std::string m_nfLabel; ///< The underlying neighborhood finder label.
    std::string m_tmLabel; ///< The topological map label.

    /// After first computation, maintain a weight map to represent the
    /// distances between decomposition regions, where the weight between cells
    /// is the euclidean distance from cell center -> shared facet center ->
    /// cell center.
    std::unordered_map<size_t, double> m_weights;

    /// The SSSP data cache.
    std::unordered_map<const WorkspaceRegion*,
        typename TopologicalFilter<MPTraits>::SSSPOutput> m_ssspCache;

    const WorkspaceRegion* m_goalRegion{nullptr}; ///< Query goal region.
    AdjacencyMap m_queryMap; ///< Query-relevant adjacency map.

    double m_backtrackDistance; ///< The distance to back track along cells.

    bool m_fallback{false}; ///< Fall back to the underlying NF on fail?

    bool m_useQueryMap{false}; ///< Use a query-relevant adjacency map?

    bool m_initialized{false}; ///< Is this object ready to use?

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
  m_weights.clear();
  m_ssspCache.clear();
}


template <typename MPTraits>
void
TopologicalFilter<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tnfLabel: " << m_nfLabel
      << "\ttmLabel: " << m_tmLabel
      << "\n\tBacktrack distance: " << m_backtrackDistance
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

  // Call the underlying NF on the reduced candidate set..
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
  throw RunTimeException(WHERE, "Not yet implemented. We need to think about "
      "how this should work - I.e., do we imply an ordering on the neighbor "
      "pairs (A, B) such that B is a topological neighbor A but not vis versa?");
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

    /// @TODO Do not use query map for PRM.

    m_ssspCache[rootRegion] = m_useQueryMap ? ComputeSSSP(rootRegion, true, m_queryMap)
                                            : ComputeSSSP(rootRegion, true);
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

  ComputeWeightMap();

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
  m_ssspCache[m_goalRegion] = ComputeSSSP(m_goalRegion);

  if(this->m_debug) {
    std::cout << "TopologicalFilter::LazyInitialize"
              << "\n\tGoal region: " << m_goalRegion
              << "\n\tDescriptor:  " << decomposition->GetDescriptor(*m_goalRegion)
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

/*----------------------------------- SSSP -----------------------------------*/

template <typename MPTraits>
void
TopologicalFilter<MPTraits>::
ComputeWeightMap() {
  MethodTimer mt(this->GetStatClass(), "TopologicalFilter::ComputeWeightMap");
  m_weights.clear();

  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto decomposition = tm->GetDecomposition();

  for(auto edge = decomposition->edges_begin();
      edge != decomposition->edges_end(); ++edge) {
    // Ensure there is exactly one facet between each pair of regions.
    const WorkspacePortal& portal = edge->property();
    auto facets = portal.FindFacets();
    if(facets.size() > 1)
      throw RunTimeException(WHERE, "This implementation assumes that the "
          "WorkspaceRegions are connected by only one facet. At this time we do "
          "not have any decompositions that need more than this. We can easily "
          "extend this by searching through the possible lengths and choosing "
          "the smallest.");

    // Get the center points of each region and the facet.
    const Point3d facet  = facets[0]->FindCenter(),
                  source = portal.GetSource().FindCenter(),
                  target = portal.GetTarget().FindCenter();

    // Compute the map key and value.
    const double distance = (target - facet).norm() + (facet - source).norm();

    m_weights[edge->id()] = distance;
  }
}


template <typename MPTraits>
typename TopologicalFilter<MPTraits>::SSSPOutput
TopologicalFilter<MPTraits>::
ComputeSSSP(const WorkspaceRegion* const _region, const bool _earlyStop,
    const AdjacencyMap& _adjacency) {
  MethodTimer mt(this->GetStatClass(), "TopologicalFilter::ComputeSSSP");
  const bool customAdjacency = !_adjacency.empty();

  if(this->m_debug)
    std::cout << "TopologicalFilter::ComputeSSSP\n";

  SSSPOutput output;
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto decomposition = tm->GetDecomposition();


  /// An element in the PQ for dijkstra's, representing one instance of
  /// discovering a cell. Cells may be discovered multiple times from different
  /// parents with different distances.
  struct element {

    VD parent;         ///< The parent cell descriptor.
    VD vd;             ///< This cell descriptor.
    double distance;   ///< Computed distance at the time of insertion.

    /// Total ordering by decreasing distance.
    bool operator>(const element& _e) const noexcept {
      return distance > _e.distance;
    }

  };

  // Define a min priority queue for dijkstras. We will not update elements when
  // better distances are found - instead we will track the most up-to-date
  // distance and ignore elements with different values. This is effectively a
  // lazy delete of stale elements.
  std::priority_queue<element,
                      std::vector<element>,
                      std::greater<element>> pq;

  // Initialize visited and temporary distance maps. The later holds an *exact*
  // copy of the most up-to-date distance for each node.
  std::unordered_map<VD, bool> visited;
  std::unordered_map<VD, double> distance;
  for(auto iter = decomposition->begin(); iter != decomposition->end(); ++iter) {
    visited[iter->descriptor()] = false;
    distance[iter->descriptor()] = std::numeric_limits<double>::max();
  }

  // Define a relax edge function.
  auto relax = [&distance, this, &pq](const ED& _ed) {
    const double sourceDistance = distance[_ed.source()],
                 targetDistance = distance[_ed.target()],
                 edgeWeight     = this->m_weights[_ed.id()],
                 newDistance    = sourceDistance + edgeWeight;

    // If the new distance isn't better, quit.
    if(newDistance >= targetDistance)
      return;

    // Otherwise, update target distance and add the target to the queue.
    distance[_ed.target()] = newDistance;
    pq.push(element{_ed.source(), _ed.target(), newDistance});
  };

  // Initialize the first node.
  const VD root = decomposition->GetDescriptor(*_region);
  distance[root] = 0;
  pq.push(element{root, root, 0});

  double maxDistance = std::numeric_limits<double>::max();
  bool foundFirstPopulated = false;

  // Dijkstras.
  while(!pq.empty()) {
    // Get the next element.
    element current = pq.top();
    pq.pop();

    // If we are done with this node, the element is stale. Discard.
    if(visited[current.vd])
      continue;
    visited[current.vd] = true;

    // Save this score and successor relationship.
    output.ordering.push_back(current.vd);
    output.distances[current.vd] = distance[current.vd];
    output.successors[current.parent].push_back(current.vd);

    if(this->m_debug)
      std::cout << "\tVertex " << current.vd
                << " has parent " << current.parent
                << ", score " << std::setprecision(4) << distance[current.vd]
                << ", and center at "
                << decomposition->GetRegion(current.vd).FindCenter()
                << std::endl;

    // Manage early stop conditions.
    if(_earlyStop) {
      // If we haven't found the first populated region yet, check for it now.
      if(!foundFirstPopulated) {
        auto& region = decomposition->GetRegion(current.vd);
        if(tm->IsPopulated(&region)) {
          foundFirstPopulated = true;
          maxDistance = distance[current.vd] + m_backtrackDistance;

          if(this->m_debug)
            std::cout << "\t\tFirst populated node found, max distance is "
                      << std::setprecision(4) << maxDistance << "."
                      << std::endl;
        }
      }
      // Otherwise, check for exceeding the max distance.
      else if(distance[current.vd] > maxDistance) {
        if(this->m_debug)
          std::cout << "\t\tLast populated node found."
                    << std::endl;
        break;
      }
    }


    // Relax each outgoing edge.
    auto vertexIter = decomposition->find_vertex(current.vd);
    for(auto edgeIter = vertexIter->begin(); edgeIter != vertexIter->end();
        ++edgeIter) {
      // If we are not using custom adjacency, simply relax the edge.
      if(!customAdjacency)
        relax(edgeIter->descriptor());
      // Otherwise, only relax if this edge appears in _adjacency.
      else if(_adjacency.count(current.vd)) {
        const auto& neighbors = _adjacency.at(current.vd);
        auto iter = std::find(neighbors.begin(), neighbors.end(),
            edgeIter->target());
        if(iter != neighbors.end())
          relax(edgeIter->descriptor());
      }
    }
  }

  // The root was added to its own successor map - fix that now.
  auto& rootMap = output.successors[root];
  rootMap.erase(std::find(rootMap.begin(), rootMap.end(), root));

  // The root was added to the front of the distance set, which we want.

  return output;
}

/*----------------------------------------------------------------------------*/

#endif
