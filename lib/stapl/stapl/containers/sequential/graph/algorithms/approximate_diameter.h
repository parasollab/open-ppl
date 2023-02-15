/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_APPROXIMATE_DIAMETER_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_APPROXIMATE_DIAMETER_HPP

#include <vector>
#include <limits>
#include <algorithm>
#include <functional>
#include "dijkstra.h"
#include "graph_algo_util.h"
#include "connected_components.h"

namespace stapl {
namespace sequential{

namespace approximate_diameter_detail {

//////////////////////////////////////////////////////////////////
/// @brief Initializes the necessary parameters and forwards other
/// parameters to Dijkstra's algorithm.
/// @param g the graph
/// @param p, d, w property maps that will hold the initial vertex
/// descriptors, the maximum weight, and the actual weight, respectively
/// @param source the source vertex whose diameter is to be found
/// @param compare the work function used for comparison of edge weights
/// @param combine the work function used for combinations
/// @param weight_max the maximum weight to be set in @p w
/// @note We use this because we need to pass a combine function to
/// Dijkstra without the queue. The initialization of that queue is
/// done here.
/// @ingroup seqGraphUtil
//////////////////////////////////////////////////////////////////
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

  dijkstra_sssp(g, p, d, w, source, compare, combine, cmap, queue);
}

////////////////////////////////////////////////////////////////////
/// @brief Iterates through the vertices of a connected component and
/// finds the vertex with the largest value in @p distmap.
/// @param cc the connected component
/// @param distmap the property map that holds the distance values
/// @param compare the work function that performs the comparison of edge
/// weights
/// @return a pair of values: the vertex with the largest distance
/// and its distance value.
/// @ingroup seqGraphAlgoWf
//////////////////////////////////////////////////////////////////////
template<typename CC, typename DMap, typename Compare>
std::pair<typename CC::value_type, typename DMap::property_value_type>
find_max_dist(CC& cc, DMap& distmap, Compare compare)
{
  typedef typename CC::value_type            vd_type;
  typedef typename DMap::property_value_type dist_type;
  typedef std::pair<vd_type, dist_type>      result_type;

  typename CC::iterator it = cc.begin(), it_end = cc.end();
  result_type far_vd = make_pair(*it, distmap.get(*it));
  ++it;
  for (; it != it_end; ++it) {
    if (compare(far_vd.second, distmap.get(*it)))
      far_vd = make_pair(*it, distmap.get(*it));
  }

  return far_vd;
}

} //namespace approximate_diameter_detail

////////////////////////////////////////////////////////////////////////////
/// @brief Approximates the diameter of the given graph.
/// @param g the graph object containing @p v
/// @param v the representative of the connected component whose diameter is to
/// be found
/// @param weightmap the property map of the edge weights to be used
/// @param compare comparison functor for the edge weights
/// @param combine combination functor for the edge weights (needed for
/// Dijkstra's algo)
/// @param max a 'max' edge weight (needed for Dijkstra's algo)
/// @return the approximate diameter of the connected component
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////////
template<typename Graph, typename WMap, typename Compare, typename Combine>
typename WMap::property_value_type
approximate_diameter(Graph& g,
                     typename Graph::vertex_descriptor v,
                     WMap& weightmap,
                     Compare compare,
                     Combine combine,
                     typename WMap::property_value_type max)
{
  typedef typename Graph::vertex_descriptor  vd_type;
  typedef typename WMap::property_value_type weight_type;
  typedef std::vector<vd_type>               cc_type;
  typedef std::pair<vd_type, weight_type>    pair_type;

  cc_type cc;
  { //scope to lose the colormap, free up some memory
  map_property_map<Graph, size_t> colormap;
  get_cc(g,colormap,v,cc);
  }

  // Find the farthest vertex from the current source, take the far
  // vertex as the new source, and repeat until the length from the
  // source to the farthest vertex stops increasing.
  //
  // Note that this can be sped up by a constant number of traversals
  // by forming a BFS visitor that performs Dijkstra's while keeping track
  // of the farthest vertex from the source at the same time. Right now
  // we call Dijkstra's then traverse the distmap.
  bool increased = true;
  map_property_map<Graph, vd_type>     predmap;
  map_property_map<Graph, weight_type> distmap;
  pair_type current = make_pair(v,weight_type()), temp;
  do {
    approximate_diameter_detail::_fwd_dsssp(g,predmap,distmap,weightmap,
                                            current.first,compare,combine,max);

    temp = approximate_diameter_detail::find_max_dist(cc,distmap,compare);
    if (compare(current.second, temp.second)) current = temp;
    else increased = false;
  } while (increased);

  return current.second;
}

//////////////////////////////////////////////////////////////////////////
/// @brief Approximates the diameter of the given graph with a default combine
/// work function.
/// @param g the graph object containing @p v
/// @param v the representative of the connected component whose diameter is to
/// be found
/// @param weightmap the property map of the edge weights to be used
/// @param compare comparison functor for the edge weights
/// @param max initialized to infinity if not specified
/// @return the approximate diameter of the connected component
/// @note the combine work function is initialized to std::plus
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template<typename Graph, typename WMap, typename Compare>
typename WMap::property_value_type
approximate_diameter(Graph& g,
                     typename Graph::vertex_descriptor v,
                     WMap& weightmap,
                     Compare compare,
                     typename WMap::property_value_type max
                     = std::numeric_limits<
                         typename WMap::property_value_type
                       >::max())
{
  std::plus<typename WMap::property_value_type> combine;
  return approximate_diameter(g,v,weightmap,compare,combine,max);
}

////////////////////////////////////////////////////////////////////////
/// @brief Approximates the diameter of the given graph with default compare
/// and combine work functions.
/// @param g the graph object containing @p v
/// @param v the representative of the connected component whose diameter is to
/// be found
/// @param weightmap the property map of the edge weights to be used
/// @param max initialized to infinity if not specified
/// @return the approximate diameter of the connected component
/// @note the compare work function is initialized to std::less and the
/// combine work function is initialized to std::plus
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////////
template<typename Graph, typename WMap>
typename WMap::property_value_type
approximate_diameter(Graph& g,
                     typename Graph::vertex_descriptor v,
                     WMap& weightmap,
                     typename WMap::property_value_type max
                     = std::numeric_limits<
                         typename WMap::property_value_type
                       >::max())
{
  std::less<typename WMap::property_value_type> lsop;
  return approximate_diameter(g,v,weightmap,lsop,max);
}

//////////////////////////////////////////////////////////////////////
/// @brief Approximates the diameter of the given graph without a provided
/// weight map and uses default compare and combine work functions.
/// @param g the graph object containing @p v
/// @param v the representative of the connected component whose diameter is to
/// be found
/// @return the approximate diameter of the connected component
/// @note the compare work function is initialized to std::less and the
/// combine work function is initialized to std::plus
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////////
template<typename Graph>
size_t approximate_diameter(Graph& g, typename Graph::vertex_descriptor v)
{
  //no weightmap given, so we'll use the number of edges along the path
  //to determine the diameter.
  map_property_map<Graph, size_t> weightmap;
  typename Graph::edge_iterator ei, ei_end;
  ei = g.edges_begin(); ei_end = g.edges_end();
  for (; ei != ei_end; ++ei) weightmap.put(*ei, 1);

  return approximate_diameter(g,v,weightmap);
}

} //namespace sequential
} //namespace stapl

#endif
