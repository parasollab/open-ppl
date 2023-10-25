/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_DIAMETER_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_DIAMETER_HPP

#include <vector>
#include <limits>
#include <algorithm>
#include <functional>
#include "dijkstra.h"
#include "graph_algo_util.h"
#include "connected_components.h"

namespace stapl {
namespace sequential{

namespace diameter_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Initializes necessary parameters and forwards other parameters
/// to Dijkstra's algorithm.
/// @param g the graph
/// @param p, d, w property maps that hold the vertex descriptors, distance
/// values and edge weights for each vertex, respectively.
/// @param source the source vertex whose diameter should be calculated
/// @param compare the comparison work function
/// @param combine the combining work function
/// @param weight_max the maximum weight value
/// @note We use this because we need to pass a combine function to Dijkstra's
/// algorithm but do not need to take a queue.
/// @ingroup seqGraphUtil
///////////////////////////////////////////////////////////////////////
template<typename Graph, typename PMap, typename DMap,
         typename WMap, typename Compare, typename Combine>
void _fwd_dsssp(Graph& g, PMap& p, DMap& d, WMap& w,
                typename Graph::vertex_descriptor source,
                Compare compare, Combine combine,
                typename WMap::property_value_type weight_max)
{
  typedef typename Graph::vertex_iterator    VI;
  typedef typename Graph::vertex_reference   VR;
  typedef typename WMap::property_value_type Weight;
  typedef map_property_map<Graph, size_t>    index_map_t;

  map_property_map<Graph, size_t> cmap;
  index_map_t index_map;
  d_ary_heap_indirect<VR, 4, index_map_t, DMap, Compare>
    queue(d, index_map, compare);

  VI vi = g.begin(); VI vi_end = g.end();
  for (; vi != vi_end; ++vi) {
    VR v = *vi;
    d.put(v, weight_max);
    p.put(v, v.descriptor());
    cmap.put(v, graph_color<size_t>::white());
  }
  d.put(source, Weight());

  dijkstra_sssp(g, p, d, w, source, weight_max, compare, combine, cmap, queue);
}

///////////////////////////////////////////////////////////////////////////
/// @brief Iterates through the vertices in a connected component and finds
/// the vertex whose distmap value is the largest.
/// @param cc the connected component
/// @param distmap the map of distance values
/// @param comp the comparison work function
/// @return the largest distance value
/// @ingroup seqGraphAlgoWf
/////////////////////////////////////////////////////////////////////////
template<typename CC, typename DMap, typename Comp>
typename DMap::property_value_type
find_max_dist(CC& cc, DMap& distmap, Comp comp)
{
  typename CC::iterator it, it_end;
  it = cc.begin(); it_end = cc.end();
  typename DMap::property_value_type result = distmap.get(*it);
  for (; it != it_end; ++it) {
    if (comp(result, distmap.get(*it))) {
      result = distmap.get(*it);
    }
  }

  return result;
}

} //namespace diameter_detail

////////////////////////////////////////////////////////////////////////////////
/// @brief Compute the diameter of the connected component represented by v.
/// @param g the graph
/// @param v the representative of the connected component whose diameter is to
///  be found
/// @param weightmap property map of the edge weights
/// @param compare the comparison functor for the edge weights
/// @param combine the combining work function for the edge weights
/// @param max the maximum edge weight needed for Dijkstra's algorithm
/// @return the diameter of the connected component
/// @note Due to the usage of Dijkstra's algorithm, the restrictions imposed by
/// Dijkstra's algorithm also apply here.
/// @note The complexity of the algorithm is O(|V| * (|E|+|V|log(|V|))).
/// @ingroup seqGraphAlgo
////////////////////////////////////////////////////////////////////////////////
template<typename Graph, typename WMap, typename Compare, typename Combine>
typename WMap::property_value_type
diameter(Graph& g,
         typename Graph::vertex_descriptor v,
         WMap& weightmap,
         Compare compare,
         Combine combine,
         typename WMap::property_value_type max)
{
  //CC <= connected component containing @v
  typedef typename WMap::property_value_type value_type;
  typedef typename Graph::vertex_descriptor  vd_type;
  typedef std::vector<vd_type>               cc_type;
  typedef map_property_map<Graph, size_t>    color_map_type;
  cc_type cc;
  {
  // scope to deallocate cmap after we've gotten the cc;
  // it's no longer necessary.
  color_map_type cmap;
  get_cc(g, cmap, v, cc);
  }

  //then compute all-pairs, shortest paths on the CC
  typedef map_property_map<Graph, vd_type>    predmap_type;
  typedef map_property_map<Graph, value_type> distmap_type;
  predmap_type predmap;
  distmap_type distmap;

  std::vector<value_type> paths;
  typename cc_type::iterator it, it_end;
  it = cc.begin(); it_end = cc.end();
  for (; it != it_end; ++it) {
    //compute all-pairs, shortest paths by calling Dijkstra's on each vertex
    diameter_detail::_fwd_dsssp(g, predmap, distmap, weightmap,
                                (*it), compare, combine, max);

    //find and keep the longest of the shortest paths from *it
    paths.push_back(diameter_detail::find_max_dist(cc, distmap, compare));
  }

  //print_vector("exact paths",paths);

  //return the longest of the longest of the shortest paths
  return *std::max_element(paths.begin(), paths.end(), compare);
}

///////////////////////////////////////////////////////////////////////////////
/// @brief Compute the diameter of the connected component represented by v
/// with a default combination function and max value.
/// @param g the graph
/// @param v the representative of the connected component whose diameter is
/// to be found
/// @param weightmap property map of the edge weights
/// @param compare the comparison functor for the edge weights
/// @param max initialized to infinity
/// @return the diameter of the connected component
/// @note Due to the usage of Dijkstra's algorithm, the restrictions imposed by
/// Dijkstra's algorithm also apply here.
/// @note The complexity of the algorithm is O(|V| * (|E|+|V|log(|V|))).
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////////
template<typename Graph, typename WMap, typename Compare>
typename WMap::property_value_type
diameter(Graph& g,
         typename Graph::vertex_descriptor v,
         WMap& weightmap,
         Compare compare,
         typename WMap::property_value_type max
         = std::numeric_limits<typename WMap::property_value_type>::max())
{
  std::plus<typename WMap::property_value_type> combine;
  return diameter(g,v,weightmap,compare,combine,max);
}

/////////////////////////////////////////////////////////////////////////////
/// @brief Compute the diameter of the connected component represented by v
/// with default comparison and combination functions and a default max value.
/// @param g the graph
/// @param v the representative of the connected component whose diameter is
/// to be found
/// @param weightmap property map of the edge weights
/// @param max initialized to infinity
/// @return the diameter of the connected component
/// @note Due to the usage of Dijkstra's algorithm, the restrictions imposed by
/// Dijkstra's algorithm also apply here.
/// @note The complexity of the algorithm is O(|V| * (|E|+|V|log(|V|))).
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////////
template<typename Graph, typename WMap>
typename WMap::property_value_type
diameter(Graph& g,
         typename Graph::vertex_descriptor v,
         WMap& weightmap,
         typename WMap::property_value_type max
         = std::numeric_limits<typename WMap::property_value_type>::max())
{
  std::less<typename WMap::property_value_type> compare;
  return diameter(g,v,weightmap,compare,max);
}


//Finds the diameter of the connected component containing v using the
//number of edges as the edge weights.

///////////////////////////////////////////////////////////////////////////////
/// @brief Compute the diameter of the connected component represented by v
/// with default comparison and combination functions, a default max value, and
/// using the number of edges as the edge weights.
/// @param g the graph
/// @param v the representative of the connected component whose diameter is
/// to be found
/// @return the diameter of the connected component
/// @note Due to the usage of Dijkstra's algorithm, the restrictions imposed by
/// Dijkstra's algorithm also apply here.
/// @note The complexity of the algorithm is O(|V| * (|E|+|V|log(|V|))).
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////////
template<typename Graph>
size_t diameter(Graph& g, typename Graph::vertex_descriptor v) {
  //no weightmap given, so we'll use the number of edges along the path
  //to determine the diameter.
  map_property_map<Graph, size_t> weightmap;
  typename Graph::edge_iterator ei, ei_end;
  ei = g.edges_begin(); ei_end = g.edges_end();
  for (; ei != ei_end; ++ei) weightmap.put(*ei, 1);

  return diameter(g,v,weightmap);
}

}//namespace sequential
} //namespace stapl

#endif
