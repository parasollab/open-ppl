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
/// @todo Move the SSSP cache into the topological map so that
///       TopologicalDistance can share it. This is not a major performance hit,
///       but (a) on principle we should never duplicate data, and (b) it could
///       become seriously detrimental if used with a very large mesh (i.e., a
///       fine tetrahedralization with many small cells).
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
    typedef WorkspaceDecomposition::vertex_descriptor VD;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    typedef std::unordered_set<VID> VertexSet;

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

    /// Hax.
    virtual void FindNeighbors(RoadmapType* _r, const CfgType& _cfg,
        const VertexSet& _candidates, std::vector<Neighbor>& _out) override;

    /// Filter the candidate range. Pass only the topologically relevant
    /// candidates to the underlying NF.
    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(RoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(RoadmapType* _r,
        InputIterator _first1, InputIterator _last1,
        InputIterator _first2, InputIterator _last2,
        OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(GroupRoadmapType* _r,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const GroupCfgType& _cfg, OutputIterator _out);

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighborPairs(GroupRoadmapType* _r,
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
    /// @param _inputCandidates The set of allowed candidates, or the roadmap if
    ///                         empty.
    /// @return The set of regions that hold candidate neighbors for _bodyIndex
    ///         at _cfg.
    PopulationMarkers FindCandidateRegions(const CfgType& _cfg,
        const size_t _bodyIndex, const VertexSet& _inputCandidates);

    /// Find the topological candidate vertices for a given configuration.
    /// @param _query The configuration we wish to connect to.
    /// @param _inputCandidates The set of allowed candidates, or the roadmap if
    ///                         empty.
    /// @return The set of VIDs that are good topological candidates for _query.
    std::vector<VID> FindCandidates(const CfgType& _cfg,
        const VertexSet& _inputCandidates = {});

    /// Experimental descriptor-counting impl.
    std::vector<VID> FindCandidatesNew(const CfgType& _cfg,
        const VertexSet& _inputCandidates = {});

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

    /// Return VIDs which are only in some of the body's frontiers?
    bool m_partialMatches{true};

    /// Try to find the nearest neighborhood for queries in obstacle space?
    bool m_recoverObstSamples{false};

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

  m_partialMatches = _node.Read("partialMatches", false, m_partialMatches,
      "Return the best partial matches when no VIDs satisfy all of the robot's "
      "bodies?");

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

/*----------------------- NeighborhoodFinder Functions -----------------------*/

template <typename MPTraits>
void
TopologicalFilter<MPTraits>::
FindNeighbors(RoadmapType* _r, const CfgType& _cfg,
    const std::unordered_set<VID>& _candidates, std::vector<Neighbor>& _out) {
  if(!m_initialized)
    LazyInitialize();

  auto stats = this->GetStatClass();

  // This object only works on the free space roadmap right now. It could be
  // expanded to handle other maps if we want, but for now we will crash if this
  // isn't the free space.
  if(_r != this->GetRoadmap())
    throw RunTimeException(WHERE) << "Only works on the free space at this time.";

  // Track the average input size.
  const size_t inputSize = _candidates.size();
  stats->GetAverage("TopologicalFilter::InputSize") += inputSize;

  if(this->m_debug)
    std::cout << "TopologicalFilter::FindNeighbors\n";

  auto nf = this->GetNeighborhoodFinder(m_nfLabel);

  // Find the topological candidate vertices.
  std::vector<VID> topologicalCandidates = FindCandidatesNew(_cfg, _candidates);
  const size_t numTopologicalCandidates = topologicalCandidates.size();
  stats->GetAverage("TopologicalFilter::TopologicalCandidates") +=
      numTopologicalCandidates;

  // Find the vertices that are in both the input set and topological
  // candidates.
  std::vector<VID> candidates;
  candidates = std::move(topologicalCandidates);

  // If we found no candidates, report fail or fall back to underlying NF.
  if(candidates.empty()) {
    // Distinguish between the two types of no-candidate scenarios.
    if(numTopologicalCandidates == 0) {
      stats->IncStat("TopologicalFilter::NoTopologicalCandidates");

      if(this->m_debug)
        std::cout << "\tNo vertices found in candidate cells."
                  << std::endl;
    }
    else {
      stats->IncStat("TopologicalFilter::NoCommonCandidates");

      if(this->m_debug)
        std::cout << "\tFound " << topologicalCandidates.size() << " vertices "
                  << "in candidate cells, but none were in the input range."
                  << std::endl;
    }

    // Fall back to underlying NF if that option is selected.
    if(m_fallback) {
      if(this->m_debug)
        std::cout << "\tFalling back to underlying nf '" << m_nfLabel << "'."
                  << std::endl;

      nf->FindNeighbors(_r, _cfg, _candidates, _out);
    }
    return;
  }

  // Call the underlying NF on the reduced candidate set.
  nf->FindNeighbors(_r, candidates.begin(), candidates.end(),
      candidates.size() == this->GetRoadmap()->Size(),
      _cfg, std::back_inserter(_out));

  // Track information on average candidate set size.
  stats->GetAverage("TopologicalFilter::UsedCandidates") += candidates.size();

  if(this->m_debug)
    std::cout << "Used candidate set."
              << "\n\t|Input Vertices|: " << inputSize
              << "\n\t|Candidates Vertices|: " << candidates.size()
              << std::endl;
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
TopologicalFilter<MPTraits>::
FindNeighbors(RoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  if(!m_initialized)
    LazyInitialize();

  auto stats = this->GetStatClass();

  // This object only works on the free space roadmap right now. It could be
  // expanded to handle other maps if we want, but for now we will crash if this
  // isn't the free space.
  if(_r != this->GetRoadmap())
    throw RunTimeException(WHERE) << "Only works on the free space at this time.";

  // Track the average input size.
  const size_t inputSize = std::distance(_first, _last);
  stats->GetAverage("TopologicalFilter::InputSize") += inputSize;

  if(this->m_debug)
    std::cout << "TopologicalFilter::FindNeighbors\n";

  auto nf = this->GetNeighborhoodFinder(m_nfLabel);

  // Find the topological candidate vertices.
  std::vector<VID> topologicalCandidates = FindCandidatesNew(_cfg);
  const size_t numTopologicalCandidates = topologicalCandidates.size();
  stats->GetAverage("TopologicalFilter::TopologicalCandidates") +=
      numTopologicalCandidates;

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
    if(numTopologicalCandidates == 0) {
      stats->IncStat("TopologicalFilter::NoTopologicalCandidates");

      if(this->m_debug)
        std::cout << "\tNo vertices found in candidate cells."
                  << std::endl;
    }
    else {
      stats->IncStat("TopologicalFilter::NoCommonCandidates");

      if(this->m_debug)
        std::cout << "\tFound " << topologicalCandidates.size() << " vertices "
                  << "in candidate cells, but none were in the input range."
                  << std::endl;
    }

    // Fall back to underlying NF if that option is selected.
    if(m_fallback) {
      if(this->m_debug)
        std::cout << "\tFalling back to underlying nf '" << m_nfLabel << "'."
                  << std::endl;

      nf->FindNeighbors(_r, _first, _last, _fromFullRoadmap, _cfg, _out);
    }
    return _out;
  }

  // Call the underlying NF on the reduced candidate set.
  nf->FindNeighbors(_r, candidates.begin(), candidates.end(),
      candidates.size() == this->GetRoadmap()->get_num_vertices(),
      _cfg, _out);

  // Track information on average candidate set size.
  stats->GetAverage("TopologicalFilter::UsedCandidates") += candidates.size();

  if(this->m_debug)
    std::cout << "Used candidate set."
              << "\n\t|Input Vertices|: " << inputSize
              << "\n\t|Candidates Vertices|: " << candidates.size()
              << std::endl;

  return _out;
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
TopologicalFilter<MPTraits>::
FindNeighborPairs(RoadmapType* _r,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  if(!m_initialized)
    LazyInitialize();

  if(this->m_debug)
    std::cout << "TopologicalFilter::FindNeighborPairs\n";

  auto g = _r;

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
    this->FindNeighbors(_r, _first2, _last2, false, g->GetVertex(*iter),
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
FindNeighbors(GroupRoadmapType* _r,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const GroupCfgType& _cfg, OutputIterator _out) {
  throw NotImplementedException(WHERE);
}


template <typename MPTraits>
template <typename InputIterator, typename OutputIterator>
OutputIterator
TopologicalFilter<MPTraits>::
FindNeighborPairs(GroupRoadmapType* _r,
    InputIterator _first1, InputIterator _last1,
    InputIterator _first2, InputIterator _last2,
    OutputIterator _out) {
  throw NotImplementedException(WHERE);
}

/*---------------------------- Candidate Neighbors ---------------------------*/

template <typename MPTraits>
typename TopologicalFilter<MPTraits>::PopulationMarkers
TopologicalFilter<MPTraits>::
FindCandidateRegions(const CfgType& _cfg, const size_t _bodyIndex,
    const VertexSet& _inputCandidates) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "TopologicalFilter::FindCandidateRegions");

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
      stats->IncStat("TopologicalFilter::NoRegion");

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
  // If we have no input candidates, then we only care about finding the first
  // populated cell for this body.
  if(_inputCandidates.empty()) {
    for(; markers.first != ordering.end(); ++markers.first) {
      // Get the next cell.
      auto& region = decomposition->GetRegion(*markers.first);
      // If it is populated, markers.first is in the right spot.
      if(tm->IsPopulated(&region, _bodyIndex))
        break;
    }
  }
  // Otherwise, we need to find the first populated cell with a valid
  // candidate.
  else {
    bool found = false;
    for(; markers.first != ordering.end(); ++markers.first) {
      // Get the VIDs in the next cell.
      auto& region = decomposition->GetRegion(*markers.first);
      const std::vector<VID> cellVIDs = tm->GetMappedVIDs(&region, _bodyIndex);

      // Check whether any of the cell VIDs are
      for(const auto vid : cellVIDs) {
        found |= _inputCandidates.count(vid);
        if(found)
          break;
      }
      if(found)
        break;
    }
  }

  // Check for no markers. This is a hard exception for now but could feasibly
  // be considered a soft-error. Usually we should not be searching for
  // neighbors until we know there are nodes to search, however.
  if(markers.first == markers.second) {
    throw RunTimeException(WHERE) << "No populated cells found!" << std::endl;
    return markers;
  }

  // Compute the max distance between the first and second marker.
  /// @todo Do we want to use larger frontiers for parts that are farther away?
  ///       This might help keep things 'centered' on the robot base.
  const double maxDistance = distance.at(*markers.first)
                           + m_backtrackDistance;

  if(this->m_debug)
    std::cout << "\t\t\tFirst populated cell " << *markers.first
              << " in order " << std::distance(ordering.begin(), markers.first)
              << " has distance "
              << std::setprecision(4) << distance.at(*markers.first) << "."
              << "\n\t\t\tSearching for new last cell with max distance "
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
std::vector<typename MPTraits::RoadmapType::VID>
TopologicalFilter<MPTraits>::
FindCandidatesNew(const CfgType& _cfg, const VertexSet& _inputCandidates) {
  MethodTimer mt(this->GetStatClass(), "TopologicalFilter::FindCandidates");

  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto mb = this->GetTask()->GetRobot()->GetMultiBody();

  std::vector<VID> newCandidates;

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
    const PopulationMarkers regions = FindCandidateRegions(_cfg, i,
        _inputCandidates);

    // Get the sorted candidates for this region.
    newCandidates.clear();
    newCandidates = tm->GetMappedVIDs(regions.first, regions.second, i);

    if(this->m_debug)
      std::cout << "\t" << newCandidates.size()
                << " candidate VIDs found for body " << i << "."
                << std::endl;

    // If we are selecting only from a set of input candidates, discard any new
    // candidates that aren't in the input set.
    if(_inputCandidates.size()) {
      for(auto iter = newCandidates.begin(); iter != newCandidates.end(); ) {
        // If this vertex is allowed, move on.
        if(_inputCandidates.count(*iter))
          ++iter;
        else
          iter = newCandidates.erase(iter);
      }

      if(this->m_debug)
        std::cout << "\t" << newCandidates.size()
                  << " candidates found in the input set."
                  << std::endl;
    }

    // Count the new candidates.
    for(const auto vid : newCandidates) {
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

  // We hve counted all of the descriptors in each body's frontier. Determine
  // the best set to return. Ask the underlying NF how many vertices it wants.
  const size_t desiredVertices = this->GetNeighborhoodFinder(m_nfLabel)->GetK();
  newCandidates.clear();
  for(auto iter = countSets.rbegin();
      iter != countSets.rend() and newCandidates.size() < desiredVertices;
      ++iter)
  {
    // Copy the next count set to the end of the new candidates.
    const auto& countSet = *iter;
    newCandidates.reserve(newCandidates.size() + countSet.size());
    auto oldEnd = newCandidates.end();
    newCandidates.insert(oldEnd, countSet.begin(), countSet.end());
    auto newEnd = newCandidates.end();

    /// @todo Do we need to sort the returned descriptors?
    // Sort the newly copied candidates.
    std::sort(oldEnd, newEnd);
    // Merge the two sorted lists.
    std::inplace_merge(newCandidates.begin(), oldEnd, newEnd);
  }


  if(this->m_debug)
    std::cout << "\tReturning " << newCandidates.size()
              << " candidates."
              << std::endl;
  return newCandidates;
}


template <typename MPTraits>
std::vector<typename MPTraits::RoadmapType::VID>
TopologicalFilter<MPTraits>::
FindCandidates(const CfgType& _cfg, const VertexSet& _inputCandidates) {
  MethodTimer mt(this->GetStatClass(), "TopologicalFilter::FindCandidates");

  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto mb = this->GetTask()->GetRobot()->GetMultiBody();

  std::vector<VID> candidateIntersection, candidateUnion, newCandidates, buffer;

  if(this->m_debug)
    std::cout << "Searching for candidate VIDs..." << std::endl;

  // Find the candidate configurations for each body and intersect them as we go.
  // Start with the EE and work backward so that partial matches favor nearby EE
  // positions.
  // OR
  // Start with the base and work outward because it is usually easier to match
  // base positions and move the EE as opposed to matching the EE position and
  // moving the base.
  /// @todo Figure out how to support tree-like robots here. The current impl
  ///       will only handle chains, and assumes the EE is the last body.
  bool first = true;
  for(size_t i = 0; i < mb->GetNumBodies(); ++i) {
    // Find the candidate regions for this body.
    const PopulationMarkers regions = FindCandidateRegions(_cfg, i,
        _inputCandidates);

    // Get the sorted candidates for this region.
    newCandidates.clear();
    newCandidates = tm->GetMappedVIDs(regions.first, regions.second, i);

    if(this->m_debug)
      std::cout << "\t" << newCandidates.size()
                << " candidate VIDs found for body " << i << "."
                << std::endl;

    // If we are selecting only from a set of input candidates, discard any new
    // candidates that aren't in the input set.
    if(_inputCandidates.size()) {
      for(auto iter = newCandidates.begin(); iter != newCandidates.end(); ) {
        // If this vertex is allowed, move on.
        if(_inputCandidates.count(*iter))
          ++iter;
        else
          iter = newCandidates.erase(iter);
      }

      if(this->m_debug)
        std::cout << "\t" << newCandidates.size()
                  << " candidates found in the input set."
                  << std::endl;
    }

    // Track partial matches if needed.
    if(m_partialMatches) {
      buffer.clear();
      buffer.reserve(std::max(candidateUnion.size(), newCandidates.size()));

      std::set_union(candidateUnion.begin(), candidateUnion.end(),
                     newCandidates.begin(),  newCandidates.end(),
                     std::back_inserter(buffer));
      std::swap(buffer, candidateUnion);
    }

    // For the first body, retain all candidates.
    if(first) {
      first = false;
      std::swap(candidateIntersection, newCandidates);
    }
    // For each successive body, intersect with existing candidates.
    else {
      buffer.clear();
      buffer.reserve(std::max(candidateIntersection.size(),
                              newCandidates.size()));
      std::set_intersection(
          candidateIntersection.begin(), candidateIntersection.end(),
          newCandidates.begin(),         newCandidates.end(),
          std::back_inserter(buffer));
      std::swap(buffer, candidateIntersection);

      if(this->m_debug)
        std::cout << "\t" << candidateIntersection.size()
                  << " in common with previous bodies."
                  << std::endl;
    }

    // If the candidates are empty, there are no ideal neighbors.
    if(candidateIntersection.empty())
      break;
  }

  // Check for partial match return.
  if(m_partialMatches and candidateIntersection.empty()) {
    if(this->m_debug)
      std::cout << "\tReturning " << candidateUnion.size() << " partial matches."
                << std::endl;
    return candidateUnion;
  }

  if(this->m_debug)
    std::cout << "\tReturning " << candidateIntersection.size()
              << " full matches."
              << std::endl;
  return candidateIntersection;
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

  auto g = this->GetRoadmap();

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

  m_ssspCache.clear();

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
    std::cout << "TopologicalFilter::BuildQueryMap"
              << "\n\tQuery:"
              << "\n\t\t(" << startVID << ") " << start.PrettyPrint()
              << "\n\t\t(" << goalVID  << ") " << goal.PrettyPrint()
              << "\n\tGoal region: " << goalRegion
              << "\n\tDescriptor:  "
              << decomposition->GetDescriptor(*goalRegion)
              << "\n\tMapped VIDs for base:";
    for(const auto vid : tm->GetMappedVIDs(goalRegion))
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


template <typename MPTraits>
SSSPOutput<WorkspaceDecomposition>&
TopologicalFilter<MPTraits>::
GetSSSPData(const WorkspaceRegion* _region) {
  auto tm = this->GetMPTools()->GetTopologicalMap(m_tmLabel);
  auto decomposition = tm->GetDecomposition();

  const bool cacheHit = m_ssspCache.count(_region);
  this->GetStatClass()->GetAverage("TopologicalFilter::CacheHitRate") += cacheHit;

  // Compute the distance map for this region if it is not cached.
  auto& ssspCache = m_ssspCache[_region];
  if(!cacheHit) {
    if(this->m_debug)
      std::cout << "\t\tSSSP cache for region "
                << decomposition->GetDescriptor(*_region) << " is cold."
                << std::endl;

    // Do the entire search for now. We will worry about pruning it later.
    // With no early-stop condition, the body index doesn't matter.
    ssspCache = m_useQueryMap ? tm->ComputeFrontier(_region, 0, -1, m_queryMap)
                              : tm->ComputeFrontier(_region, 0, -1);

    // Remove the data we will not use.
    ssspCache.parent.clear();
    ssspCache.successors.clear();
  }

  return ssspCache;
}

/*----------------------------------------------------------------------------*/

#endif
