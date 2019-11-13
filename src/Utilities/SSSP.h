#ifndef PMPL_SSSP_H_
#define PMPL_SSSP_H_

#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// A descriptor-based adjacency map. Can model alternative adjacency mappings
/// and successors from an SSSP run.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
using SSSPAdjacencyMap =
      std::unordered_map<typename GraphType::vertex_descriptor,
                         std::vector<typename GraphType::vertex_descriptor>>;


////////////////////////////////////////////////////////////////////////////
/// The output of an SSSP run.
////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
struct SSSPOutput {

  typedef typename GraphType::vertex_descriptor VD;
  typedef std::unordered_map<VD, double>        DistanceMap;
  typedef std::vector<VD>                       Ordering;
  typedef SSSPAdjacencyMap<GraphType>           Adjacency;
  typedef std::unordered_map<VD, VD>            ParentMap;

  DistanceMap distance;  ///< Distance to each cell from start.
  Ordering ordering;     ///< Cell discovery ordering.
  Adjacency successors;  ///< Maps predecessor -> successors.
  ParentMap parent;      ///< Maps successor -> parent.

};


////////////////////////////////////////////////////////////////////////////////
/// The possible early-termination conditions for an SSSP run.
////////////////////////////////////////////////////////////////////////////////
enum class SSSPTermination {
  Continue,   ///< Proceed as usual.
  EndBranch,  ///< End the branch at this vertex and do not relax its neighbors.
  EndSearch   ///< End the entire search process.
};


/// Debug output for termination criteria.
std::ostream& operator<<(std::ostream& _os, const SSSPTermination& _t);


////////////////////////////////////////////////////////////////////////////////
/// Define an early termination criterion as a function which takes a vertex
/// iterator and the current output object.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
using SSSPTerminationCriterion =
      std::function<SSSPTermination(typename GraphType::vertex_iterator&,
                                    const SSSPOutput<GraphType>& _sssp)>;


/// Create a standard SSSP stop criterion.
/// @return A termination criterion which never stops early.
template <typename GraphType>
SSSPTerminationCriterion<GraphType>
SSSPDefaultTermination() {
  return [](typename GraphType::vertex_iterator&, const SSSPOutput<GraphType>&)
         {
           return SSSPTermination::Continue;
         };
}


////////////////////////////////////////////////////////////////////////////////
/// Define a path weight function for SSSP. This takes an edge iterator and the
/// best path distances to the source and target nodes found so far. It should
/// return the new target path distance that would be achieved by using this
/// edge instead of the previous best. Use infinity to represent an
/// untraversable edge, or max double to represent a worst-case (but still
/// traversable) edge.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
using SSSPPathWeightFunction =
      std::function<double(typename GraphType::adj_edge_iterator&, const double,
                           const double)>;


/// Create a standard SSSP weight function.
/// @return A path weight function which adds the edge property weight to the
///         source distance.
template <typename GraphType>
SSSPPathWeightFunction<GraphType>
SSSPDefaultPathWeight() {
  return [](typename GraphType::adj_edge_iterator& _ei,
            const double _sourceDistance,
            const double _targetDistance)
         {
           const double edgeWeight  = _ei->property().GetWeight(),
                        newDistance = _sourceDistance + edgeWeight;
           return newDistance;
         };
}


/// Compute the SSSP through a graph from a start node with Dijkstra's algorithm.
/// @param _g The graph pointer.
/// @param _starts The vertex descriptors to start from.
/// @param _weight The function for determining total path weight.
/// @param _earlyStop The early termination criterion.
/// @return An output object which contains the discovered information.
template <typename GraphType>
SSSPOutput<GraphType>
DijkstraSSSP(
    GraphType* const _g,
    const std::vector<typename GraphType::vertex_descriptor>& _starts,
    SSSPPathWeightFunction<GraphType>& _weight,
    SSSPTerminationCriterion<GraphType>& _earlyStop,
    const SSSPAdjacencyMap<GraphType>& _adjacencyMap = {})
{
  static constexpr bool debug = false;
  const bool customAdjacency = !_adjacencyMap.empty();
  if(debug)
    std::cout << "DijkstraSSSP" << std::endl;

  using VD = typename GraphType::vertex_descriptor;
  using EI = typename GraphType::adj_edge_iterator;

  /// An element in the PQ for dijkstra's, representing one instance of
  /// discovering a cell. Cells may be discovered multiple times from different
  /// parents with different distances.
  struct element {

    VD parent;         ///< The parent cell descriptor.
    VD vd;             ///< This cell descriptor.
    double distance;   ///< Computed distance at the time of insertion.

    /// Construct an element for the search queue.
    /// @param _parent The parent node descriptor.
    /// @param _target The target node descriptor.
    /// @param _distance The distance from parent to target.
    element(const VD _parent, const VD _target, const double _distance)
      : parent(_parent), vd(_target), distance(_distance) {}

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
  // copy of the most up-to-date distance for each node. The absence of an entry
  // will be taken as the initial state.
  std::unordered_set<VD> visited;
  std::unordered_map<VD, double> distance;

  // Initialize the output object.
  SSSPOutput<GraphType> output;

  // Define a relax edge function.
  auto relax = [&distance, &pq, &_weight](EI& _ei) {
    const VD source = _ei->source(),
             target = _ei->target();

    const double sourceDistance = distance[source],
                 targetDistance = distance.count(target)
                                ? distance[target]
                                : std::numeric_limits<double>::infinity(),
                 newDistance    = _weight(_ei, sourceDistance, targetDistance);

    // If the new distance isn't better, quit.
    if(newDistance >= targetDistance)
      return;

    // Otherwise, update target distance and add the target to the queue.
    distance[target] = newDistance;
    pq.emplace(source, target, newDistance);
  };

  // Initialize each start node.
  for(const auto start : _starts) {
    distance[start] = 0;
    pq.emplace(start, start, 0);
  }

  // Dijkstras.
  while(!pq.empty()) {
    // Get the next element.
    element current = pq.top();
    pq.pop();

    // If we are done with this node, the element is stale. Discard.
    if(visited.count(current.vd))
      continue;
    visited.insert(current.vd);

    // Save this score and successor relationship.
    output.ordering.push_back(current.vd);
    output.distance[current.vd] = distance[current.vd];
    output.successors[current.parent].push_back(current.vd);
    output.parent[current.vd] = current.parent;

    // Check for early termination.
    auto vi = _g->find_vertex(current.vd);
    auto stop = _earlyStop(vi, output);

    if(debug)
      std::cout << "\tVertex: " << current.vd
                << ", parent: " << current.parent
                << ", score: " << std::setprecision(4) << distance[current.vd]
                << ", stop: " << stop
                << std::endl;

    if(stop == SSSPTermination::Continue)
      // Continue the search as usual.
      ;
    else if(stop == SSSPTermination::EndBranch)
      // End this branch (do not relax neighbors).
      continue;
    else if(stop == SSSPTermination::EndSearch)
      // End the entire search.
      break;

    // Relax each outgoing edge.
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      // If we are not using custom adjacency, simply relax the edge.
      if(!customAdjacency)
        relax(ei);
      // Otherwise, only relax if this edge appears in _adjacencyMap.
      else if(_adjacencyMap.count(current.vd)) {
        const auto& neighbors = _adjacencyMap.at(current.vd);
        auto iter = std::find(neighbors.begin(), neighbors.end(), ei->target());
        if(iter != neighbors.end())
          relax(ei);
      }
    }
  }

  // The _starts were added to their own successor map - fix that now.
  for(const auto start : _starts) {
    auto& startMap = output.successors[start];
    startMap.erase(std::find(startMap.begin(), startMap.end(), start));
  }

  return output;
}


/// Compute the SSSP through a graph from a start node with Dijkstra's algorithm.
/// Use the standard termination criterion.
/// @param _g The graph pointer.
/// @param _starts The vertex descriptors to start from.
/// @param _weight The function for determining total path weight.
/// @return An output object which contains the discovered information.
template <typename GraphType>
SSSPOutput<GraphType>
DijkstraSSSP(
    GraphType* const _g,
    const std::vector<typename GraphType::vertex_descriptor>& _starts,
    SSSPPathWeightFunction<GraphType>& _weight,
    const SSSPAdjacencyMap<GraphType>& _adjacencyMap = {})
{
  auto stop = SSSPDefaultTermination<GraphType>();

  return DijkstraSSSP(_g, _starts, _weight, stop, _adjacencyMap);
}


/// Compute the SSSP through a graph from a start node with Dijkstra's algorithm.
/// Use the standard path weight.
/// @param _g The graph pointer.
/// @param _starts The vertex descriptors to start from.
/// @param _stop The termination criterion.
/// @return An output object which contains the discovered information.
template <typename GraphType>
SSSPOutput<GraphType>
DijkstraSSSP(
    GraphType* const _g,
    const std::vector<typename GraphType::vertex_descriptor>& _starts,
    SSSPTerminationCriterion<GraphType>& _stop,
    const SSSPAdjacencyMap<GraphType>& _adjacencyMap = {})
{
  auto weight = SSSPDefaultPathWeight<GraphType>();

  return DijkstraSSSP(_g, _starts, weight, _stop, _adjacencyMap);
}


/// Compute the SSSP through a graph from a start node with Dijkstra's algorithm.
/// Use the standard path weight and termination criterion.
/// @param _g The graph pointer.
/// @param _starts The vertex descriptors to start from.
/// @return An output object which contains the discovered information.
template <typename GraphType>
SSSPOutput<GraphType>
DijkstraSSSP(
    GraphType* const _g,
    const std::vector<typename GraphType::vertex_descriptor>& _starts,
    const SSSPAdjacencyMap<GraphType>& _adjacencyMap = {})
{
  auto weight = SSSPDefaultPathWeight<GraphType>();
  auto stop   = SSSPDefaultTermination<GraphType>();

  return DijkstraSSSP(_g, _starts, weight, stop, _adjacencyMap);
}


#endif
