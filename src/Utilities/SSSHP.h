#ifndef _PPL_SSSHP_H_
#define _PPL_SSSHP_H_

#include "Hypergraph.h"
#include "MPUtils.h"

#include <unordered_map>

//////////////////////////////////////////////////////////////////////
/// This file contains an implementation of the Shortest B-Tree 
/// Dijkstra Algorithm described in "Directed hypergraphs and
/// applications" by Giorio Gallo, Giustino Longo, and Stefano
/// Pallottino. The input is a directed hypergraph and a single source
/// and target node. The output is the minimum B-tree rooted at the 
/// source. The shortest hyperpath from the source to the target can 
/// be extracted from this B-tree. (B-tree is defined the paper).
//////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
/// The output of an SSSHP run.
////////////////////////////////////////////////////////////////////////////
struct MBTOutput {

  typedef std::unordered_map<size_t,double> WeightMap;
  typedef std::vector<size_t>               Ordering;
  typedef std::unordered_map<size_t,size_t> VertexParentMap;
  typedef std::unordered_map<size_t,size_t> HyperarcParentMap;

  WeightMap         weightMap;
  Ordering          ordering;
  VertexParentMap   vertexParentMap;
  HyperarcParentMap hyperarcParentMap;

};


////////////////////////////////////////////////////////////////////////////////
/// The possible early-termination conditions for an SSSHP run.
////////////////////////////////////////////////////////////////////////////////
enum class SSSHPTermination {
  Continue,  ///< Proceed as usual.
  EndBranch, ///< End the branch at this vertex and do not relax the neighbors.
  EndSearch ///< End the entire search process.
};

////////////////////////////////////////////////////////////////////////////////
/// Define an early termination criterion as a function which takes a vertex
/// iterator and the current output object.
////////////////////////////////////////////////////////////////////////////////
using SSSHPTerminationCriterion = 
  std::function<SSSHPTermination(size_t& _vid, const MBTOutput& _mbt)>;

/// Create a standard SSSHP stop criterion.
/// @return A termination criterion which never stops early.
SSSHPTerminationCriterion
SSSHPDefaultTermination();

////////////////////////////////////////////////////////////////////////////////
/// Define a weight function for the hyperarcs. This takes in a hyperarc and
/// and returns a double.
////////////////////////////////////////////////////////////////////////////////
template <typename VertexType, typename HyperarcType>
using SSSHPPathWeightFunction =
  std::function<double(
    const typename Hypergraph<VertexType, HyperarcType>::Hyperarc& _hyperarc,
    const std::unordered_map<size_t,double> _weightMap,
    const size_t _target)>;

/// Create a standard SSSHP weight function. This implements the distance 
/// function described in the paper. 
/// @return A path weight function which adds the hyperarc property weight
///         to the highest weighted tail vertex of the hyperarc.
template <typename VertexType, typename HyperarcType>
SSSHPPathWeightFunction<VertexType,HyperarcType>
DefaultSSSHPPathWeightFunction() {
  return [](
       const typename Hypergraph<VertexType, HyperarcType>::Hyperarc& _hyperarc,
       const std::unordered_map<size_t,double> _weightMap,
       const size_t _target) 
         {
            const double hyperarcWeight = _hyperarc.property.GetWeight();
            double tailWeight = 0;
            
            for(auto vid : _hyperarc.tail) {
              tailWeight = std::max(tailWeight,_weightMap.at(vid));
            }

            return hyperarcWeight + tailWeight;
         };
}

////////////////////////////////////////////////////////////////////////////////
/// Define a forward star function for vertices. This takes in a vertex and
/// returns a set of outgoing hyperarcs.
////////////////////////////////////////////////////////////////////////////////
template <typename VertexType, typename HyperarcType>
using SSSHPForwardStar =
  std::function<std::set<size_t>(
    const size_t _vid,
    Hypergraph<VertexType,HyperarcType>* _h)>;

/// Create a standard SSSHP forward star function. This simply asks the
/// hypergraph for the outgoing hyperarcs.
/// @return A set of outgoing hyperarcs for the vertex.
template <typename VertexType, typename HyperarcType>
SSSHPForwardStar<VertexType,HyperarcType>
DefaultSSSHPForwardStar() {
  return [](const size_t& _vid, Hypergraph<VertexType,HyperarcType>* _h) {
    return _h->GetOutgoingHyperarcs(_vid);
  };
}

/// Compute the SSSP through a graph from a start node with Dijkstra's algorithm.
template <typename VertexType, typename HyperarcType>
MBTOutput
SBTDijkstra(
  Hypergraph<VertexType,HyperarcType>* _h,
  size_t _source,
  SSSHPPathWeightFunction<VertexType,HyperarcType> _weight,
  SSSHPTerminationCriterion _termination,
  SSSHPForwardStar<VertexType,HyperarcType> _forwardStar,
  const double _sourceWeight = 0) {

  // Set up local variables.

  // Initialize the output object.
  MBTOutput mbt;

  // Initialize tail node counts.
  std::unordered_map<size_t,size_t> tailNodeCounter; //k_j in paper
  for(const auto& kv : _h->GetHyperarcMap()) {
    tailNodeCounter[kv.first] = 0;
  }
 
  // Initialize vertex weights.
  for(const auto& kv : _h->GetVertexMap()) {
    mbt.weightMap[kv.first] = MAX_INT;
  }

  mbt.weightMap[_source] = _sourceWeight;
  
  // Initialize queue
  
  /// Queue Element
  /// This represents an instance of discovering a vertex in the
  /// Dijkstra search. A vertex may be discovered multiple times
  /// from different parent hyperarcs with different weights.
  struct element {

    size_t parent; ///< The parent hyperarc id.
    size_t vid;    ///< The vertex id.
    double weight; ///< The weight for this visit of the node.

    /// Construct an element for the seaerch queue.
    /// @param _parent The parent hyperarc id.
    /// @param _vid    The vertex id.
    /// @param _wieght The weight of this visit.
    element(const size_t _parent, const size_t _vid, const double _weight)
      : parent(_parent), vid(_vid), weight(_weight) { }

    /// Total ordering by descreasing weight.
    bool operator>(const element _e) const noexcept {
      return weight > _e.weight;
    }

  };

  // Define a min priority queue for Dijkstras.
  std::priority_queue<element,
                      std::vector<element>,
                      std::greater<element>> pq;

  // Initialize visited set
  std::set<size_t> visited;

  // Define a relax function.
  auto relax = [&mbt, &pq, &_weight](size_t _target,
          const typename Hypergraph<VertexType,HyperarcType>::Hyperarc& _hyperarc) {

    const double newWeight = _weight(_hyperarc,mbt.weightMap,_target);

    // If the new distance is not better, quit.
    if(newWeight >= mbt.weightMap[_target])
      return;

    // Otherwise, update the target distance and add the target to the queue.
    mbt.weightMap[_target] = newWeight;
    pq.emplace(_hyperarc.hid, _target, newWeight);
  };

  // Intialize starting node in queue.
  pq.emplace(MAX_INT,_source,_sourceWeight);

  // SBT-Dijkstras
  while(!pq.empty()) {
    // Get the next element.
    element current = pq.top();
    pq.pop();

    if(current.vid == 878)
      std::cout << "HERE" << std::endl;

    // If this node has already been visited, the element is stale. Discard.
    if(visited.count(current.vid))
      continue;
    visited.insert(current.vid);

    // Save the vertex's parent hyperarc.
    mbt.vertexParentMap[current.vid] = current.parent;
    mbt.ordering.push_back(current.vid);

    // Check for early termination
    auto stop = _termination(current.vid,mbt);

    if(stop == SSSHPTermination::Continue)
      // Continue as usual.
      ;
    else if(stop == SSSHPTermination::EndBranch)
      // End this branch and do not relax it's neighbors.
      continue;
    else if(stop == SSSHPTermination::EndSearch)
      // End the entire search.
      break;

    // Iterate through the forward start of the current vertex.
    for(auto hid : _forwardStar(current.vid,_h)) {

      // Fetch the hyperarc from the hypergraph.
      const auto& hyperarc = _h->GetHyperarc(hid);

      // Increment the tail counter of the hyperarc
      tailNodeCounter[hid]++;

      // Check if all the vertices in the tail have been visited.
      if(tailNodeCounter[hid] < hyperarc.tail.size())
        continue;

      // Set hyperarc parent vertex.
      mbt.hyperarcParentMap[hid] = current.vid;

      // Relax each vertex at the head of the hyperarc.
      for(auto vid : hyperarc.head) {
        relax(vid,hyperarc);
      }
    }
  }
  
  return mbt;
}

#endif
