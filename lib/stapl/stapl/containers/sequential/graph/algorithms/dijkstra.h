/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_ALGO_DIJKSTRA_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_ALGO_DIJKSTRA_HPP

#include <vector>
#include <limits>
#ifndef __APPLE__
  #include "values.h"
#endif
#include "util/d_ary_heap.h"
#include "graph_algo_util.h"
#include <stapl/containers/type_traits/index_bounds.hpp>

namespace stapl{
namespace sequential{

/**
 * Implementation of Dijkstra's algorithm, solving the single-source,
 * shortest path problem.
 *
 * Note that the function call does not take an edge weight map;
 * the implementations access the graph's entire edge property directly,
 * so they must be internal. This means that the property on the edge
 * must have the appropriate compare and combine functors defined for
 * it. In the default case, this means '<' and '+' operators. So if
 * the edge weights are some class foo, foo+foo and foo<foo (returning bool)
 * need to be defined, and the result of foo+foo needs to be the
 * property_value_type of the distance map output parameter.
 */

///////////////////////////////////////////////////////////////////////////
/// @brief Returns the property of a vertex, given the graph
/// and the vertex descriptor.
/// @param g The graph containing the vertex.
/// @param vd The vertex descriptor.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
///////////////////////////////////////////////////////////////////////////
template<typename GRAPH>
typename GRAPH::vertex_property get_vertex_data(const GRAPH& g,
                          typename GRAPH::vertex_descriptor vd)
{
  typename GRAPH::const_vertex_iterator vi = g.find_vertex(vd);
  return (*vi).property();
}

///////////////////////////////////////////////////////////////////////////
/// @brief Gets an edge property, given a graph and the vertex
/// descriptors of the two endpoints of the edge.
/// @param g The input graph containing the edge.
/// @param vd1 The first vertex descriptor.
/// @param vd2 The second vertex descriptor.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
///////////////////////////////////////////////////////////////////////////
template<typename GRAPH>
typename GRAPH::edge_property get_edge_data(const GRAPH& g,
                                         typename GRAPH::vertex_descriptor vd1,
                                         typename GRAPH::vertex_descriptor vd2)
{
  typename GRAPH::const_vertex_iterator vi;
  typename GRAPH::const_adj_edge_iterator   ei;
  typename GRAPH::edge_descriptor ed(vd1,vd2);
  g.find_edge(ed,vi,ei);
  return (*ei).property();
}

///////////////////////////////////////////////////////////////////////////
/// @brief Uses Dijkstra's algorithm to calculate an SSSP tree
/// for a given graph. In the SSSP tree, the edge weights represent the
/// distance of the shortest path from the source vertex to each destination
/// (the cumulative path weights).
/// @param g The input graph.
/// @param parentmap A parent map to hold the parent of each vertex v in the
/// shortest path from source to v.
/// @param distmap A construct to hold the distance from the source node to
/// all destination nodes.
/// @param weightmap A construct to hold the weights of the edges.
/// @param source The desired source node for the SSSP tree.
/// @param goal A vertex that, if reached, will break out early from the
/// algorithm.
/// @param comp A comparator used to decide which path weight is better.
/// @param weight_max A maximum weight for the distance.
/// @param combine A function that defines how two edge weights are combined.
/// @param colormap A color map used for this algorithm.
/// @param Q The queue used to hold the vertices.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph, typename Comparator, typename CombineFunction,
          typename PMap, typename DMap, typename WMap,
          typename CMap, typename Queue>
void dijkstra_sssp(Graph& g,
                   PMap& parentmap,
                   DMap& distmap,
                   WMap& weightmap,
                   typename Graph::vertex_descriptor source,
                   typename Graph::vertex_descriptor goal,
                   Comparator& comp,
                   typename WMap::property_value_type weight_max,
                   CombineFunction& combine,
                   CMap& colormap,
                   Queue& Q)
{
  typedef typename Graph::vertex_iterator   VI;
  typedef typename Graph::vertex_descriptor VD;
  typedef typename Graph::vertex_reference  VR;
  typedef typename Graph::adj_edge_iterator AEI;
  typedef typename WMap::property_value_type Weight;
  typedef typename DMap::property_value_type dist_type;
  typedef typename CMap::property_value_type  color_type;
  typedef graph_color<color_type>             color_set;

  colormap.put(source, color_set::gray());
  Q.push(*g.find_vertex(source));
  while (!Q.empty()) {
    VR u = Q.top();
    if (distmap.get(u) == weight_max)
      break;
    if (u.descriptor() == goal)
      break;
    Q.pop();
    AEI ei_end = u.end();
    AEI ei = u.begin();

    for (; ei!=ei_end; ei++) {
      VD v = (*ei).target();
      color_type color = colormap.get(v);
      if (color == color_set::white()) {
        //discovered a new vertex; relax and push into the queue.
        VI v_iter = g.find_vertex(v);
        dist_type d_u, d_v;
        d_u = distmap.get(u);
        d_v = distmap.get(*v_iter);
        const Weight& weight = weightmap.get(*ei);
        if (comp(combine(d_u, weight), d_v)) {
          distmap.put(*v_iter, combine(d_u,weight));
          parentmap.put(*v_iter, u.descriptor());
        }
        colormap.put(v, color_set::gray());
        Q.push(*v_iter);
      }
      else if (color == color_set::gray()) {
        //vertex is already in the queue; relax and update if necessary.
        VI v_iter = g.find_vertex(v);
        dist_type d_u, d_v;
        d_u = distmap.get(u);
        d_v = distmap.get(*v_iter);
        const Weight& weight = weightmap.get(*ei);
        if (comp(combine(d_u, weight), d_v)) {
          distmap.put(*v_iter, combine(d_u,weight));
          parentmap.put(*v_iter, u.descriptor());
          if (comp(distmap.get(*v_iter), d_v))
            Q.update(*v_iter);
        }
      }
    }
    colormap.put(u, color_set::black());
  }
}

///////////////////////////////////////////////////////////////////////////
/// @brief Uses Dijkstra's algorithm to generate an SSSP tree
/// for a given input graph and starting vertex. No goal vertex is specified.
/// This function uses an invalid vertex descriptor as the goal vertex,
/// effectively setting no goal vertex.
/// @param g The input graph.
/// @param parentmap A parent map to hold the parent of each vertex v in the
/// shortest path from source to v.
/// @param distmap A construct to hold the distance from the source node to
/// all destination nodes.
/// @param weightmap A construct to hold the weights of the edges.
/// @param source The desired source node for the SSSP tree.
/// @param comp A comparator used to decide which path weight is better.
/// @param weight_max A maximum weight for the distance.
/// @param combine A function that defines how two edge weights are combined.
/// @param colormap A color map used for this algorithm.
/// @param Q The queue used to hold the vertices.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph, typename Comparator, typename CombineFunction,
          typename PMap, typename DMap, typename WMap,
          typename CMap, typename Queue>
void dijkstra_sssp(Graph& g,
                   PMap& parentmap,
                   DMap& distmap,
                   WMap& weightmap,
                   typename Graph::vertex_descriptor source,
                   typename WMap::property_value_type weight_max,
                   Comparator& comp,
                   CombineFunction& combine,
                   CMap& colormap,
                   Queue& Q)
{
  typename Graph::vertex_descriptor goal =
    index_bounds<typename Graph::vertex_descriptor>::invalid();
  dijkstra_sssp(g, parentmap, distmap, weightmap, source,
                goal, comp, weight_max, combine, colormap, Q);
}
///////////////////////////////////////////////////////////////////////////
/// @brief Uses Dijkstra's algorithm to create an SSSP tree for
/// a given graph and source vertex. The parent map, distance map, and weight
/// map are assumed to not be filled. No combine method is specified, so the
/// std::plus function is used. No queue is specified, and a default queue is
/// used. No colormap is specified, so a colormap with all vertices set at
/// white is used.
/// @param g The input graph.
/// @param parentmap A parent map to hold the parent of each vertex v in the
/// shortest path from source to v.
/// @param distmap A construct to hold the distance from the source node to
/// all destination nodes.
/// @param weightmap A construct to hold the weights of the edges.
/// @param source The desired source node for the SSSP tree.
/// @param goal A vertex that, if reached, will break out early from the
/// algorithm.
/// @param comp A comparator used to decide which path weight is better.
/// @param weight_max A maximum weight for the distance.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph, typename PMap, typename DMap,
          typename WMap, typename Comparator>
void dijkstra_sssp(Graph& g,
                   PMap& parentmap,
                   DMap& distmap,
                   WMap& weightmap,
                   typename Graph::vertex_descriptor source,
                   typename Graph::vertex_descriptor goal,
                   Comparator& comp,
                   typename WMap::property_value_type weight_max)
{
  typedef typename Graph::vertex_iterator    VI;
  typedef typename Graph::vertex_reference   VR;
  typedef typename WMap::property_value_type Weight;
  typedef map_property_map<Graph, size_t>    index_map_t;

  map_property_map<Graph, size_t> colormap;
  index_map_t index_map;
  d_ary_heap_indirect<VR, 4, index_map_t, DMap, Comparator>
    queue(distmap, index_map, comp);

  VI vi = g.begin();
  VI vi_end = g.end();
  for (; vi != vi_end; ++vi) {
    VR v = *vi;
    distmap.put(v, weight_max);
    parentmap.put(v, v.descriptor());
    colormap.put(v, graph_color<size_t>::white());
  }
  distmap.put(source, Weight());

  std::plus<Weight> combine;
  dijkstra_sssp(g, parentmap, distmap, weightmap,
                source, goal, comp, weight_max,
                combine, colormap, queue);
}

///////////////////////////////////////////////////////////////////////////
/// @brief Uses Dijkstra's algorithm to create an SSSP tree for
/// a given graph and source vertex. The parent map, distance map, and weight
/// map are assumed to not be filled. No combine method is specified, so the
/// std::plus function is used. No comparator function is specified, so the
/// less than comparator is used. No queue is specified, and a default queue is
/// used. No colormap is specified, so a colormap with all vertices set at
/// white is used. No goal is specified, so an invalid vertex descriptor is
/// used as the goal to have the effect of having no goal vertex.
/// @param g The input graph.
/// @param parentmap A parent map to hold the parent of each vertex v in the
/// shortest path from source to v.
/// @param distmap A construct to hold the distance from the source node to
/// all destination nodes.
/// @param weightmap A construct to hold the weights of the edges.
/// @param source The desired source node for the SSSP tree.
/// @param weight_max A maximum weight for the distance.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph, typename PMap, typename DMap, typename WMap>
void dijkstra_sssp(Graph& g,
                   PMap& parentmap,
                   DMap& distmap,
                   WMap& weightmap,
                   typename Graph::vertex_descriptor source,
                   typename WMap::property_value_type weight_max
                   = std::numeric_limits<typename
                   WMap::property_value_type>::max())
{
  typedef typename WMap::property_value_type Weight;
  std::less<Weight> lsop;
  typename Graph::vertex_descriptor goal =
    index_bounds<typename Graph::vertex_descriptor>::invalid();

  dijkstra_sssp(g,parentmap,distmap,weightmap, source, goal, lsop,weight_max);
}

///////////////////////////////////////////////////////////////////////////
/// @brief Uses Dijkstra's algorithm to create an SSSP tree for
/// a given graph and source vertex. The parent map, distance map, and weight
/// map are assumed to not be filled. No combine method is specified, so the
/// std::plus function is used. No comparator function is specified, so the
/// less than comparator is used. No queue is specified, and a default queue is
/// used. No colormap is specified, so a colormap with all vertices set at
/// white is used.
/// @param g The input graph.
/// @param parentmap A parent map to hold the parent of each vertex v in the
/// shortest path from source to v.
/// @param distmap A construct to hold the distance from the source node to
/// all destination nodes.
/// @param weightmap A construct to hold the weights of the edges.
/// @param source The desired source node for the SSSP tree.
/// @param goal The goal vertex that, if reached, will break out of the
/// algorithm early.
/// @param weight_max A maximum weight for the distance.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph, typename PMap, typename DMap, typename WMap>
void dijkstra_sssp(Graph& g,
                   PMap& parentmap,
                   DMap& distmap,
                   WMap& weightmap,
                   typename Graph::vertex_descriptor source,
                   typename Graph::vertex_descriptor goal,
                   typename WMap::property_value_type weight_max
                   = std::numeric_limits<typename
                   WMap::property_value_type>::max())
{
  typedef typename WMap::property_value_type Weight;
  std::less<Weight> lsop;

  dijkstra_sssp(g,parentmap,distmap,weightmap, source, goal, lsop,weight_max);
}

///////////////////////////////////////////////////////////////////////////
/// @brief Uses Dijkstra's algorithm to create an SSSP tree for
/// a given graph and source vertex. The parent map and distance map are
/// assumed to not be filled. No combine method is specified, so the
/// std::plus function is used. No comparator function is specified, so the
/// less than comparator is used. No queue is specified, and a default queue is
/// used. No colormap is specified, so a colormap with all vertices set at
/// white is used. No weight map is specified, so a default weightmap with
/// weights initialized to a std::size_t with value 1 is used. No goal is
/// specified, so an invalid vertex descriptor is used as the goal to have
/// the effect of having no goal vertex.
/// @param g The input graph.
/// @param parentmap A parent map to hold the parent of each vertex v in the
/// shortest path from source to v.
/// @param dmap A construct to hold the distance from the source node to
/// all destination nodes.
/// @param source The desired source node for the SSSP tree.
/// @param weight_max A maximum weight for the distance.
/// @todo Harsh suggests that what to do w/o a weightmap param is to use a map
/// from VD to size_t, initializing it with 1. If the user's dmap value type
/// can't perform the necessary '<', '+', and conversions involving size_t,
/// then that's their problem.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph, typename PMap, typename DMap>
void dijkstra_sssp(Graph& g,
                   PMap& parentmap,
                   DMap& dmap,
                   typename Graph::vertex_descriptor source,
                   size_t weight_max = std::numeric_limits<size_t>::max())
{
  //no weightmap parameter; use map from VD to size_t, initialized to 1
  map_property_map<Graph, size_t> weightmap;
  typename Graph::vertex_descriptor goal =
    index_bounds<typename Graph::vertex_descriptor>::invalid();

  typename Graph::vertex_iterator it, it_end = g.end();
  for (it = g.begin(); it != it_end; ++it)
    weightmap.put(*it, 1);

  std::less<size_t> lsop;
  dijkstra_sssp(g, parentmap, dmap, weightmap, source, goal, lsop, weight_max);
}

///////////////////////////////////////////////////////////////////////////
/// @brief Uses Dijkstra's algorithm to create an SSSP tree for
/// a given graph and source vertex. The parent map and distance map are
/// assumed to not be filled. No combine method is specified, so the
/// std::plus function is used. No comparator function is specified, so the
/// less than comparator is used. No queue is specified, and a default queue is
/// used. No colormap is specified, so a colormap with all vertices set at
/// white is used. No weight map is specified, so a default weightmap with
/// weights initialized to a std::size_t with value 1 is used.
/// @param g The input graph.
/// @param parentmap A parent map to hold the parent of each vertex v in the
/// shortest path from source to v.
/// @param dmap A construct to hold the distance from the source node to
/// all destination nodes.
/// @param source The desired source node for the SSSP tree.
/// @param goal The goal vertex that, if reached, will break out of the
/// algorithm early.
/// @param weight_max A maximum weight for the distance.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph, typename PMap, typename DMap>
void dijkstra_sssp(Graph& g,
                   PMap& parentmap,
                   DMap& dmap,
                   typename Graph::vertex_descriptor source,
                   typename Graph::vertex_descriptor goal,
                   size_t weight_max = std::numeric_limits<size_t>::max())
{
  //no weightmap parameter; use map from VD to size_t, initialized to 1
  map_property_map<Graph, size_t> weightmap;

  typename Graph::vertex_iterator it, it_end = g.end();
  for (it = g.begin(); it != it_end; ++it)
    weightmap.put(*it, 1);

  std::less<size_t> lsop;
  dijkstra_sssp(g, parentmap, dmap, weightmap, source, goal, lsop, weight_max);
}

//FIXME: Need to discuss this interface; what if there isn't an edge property?
//Or it's composed? Do we redo the call interface like dijkstra_sssp? Or give
//weightmap params to these algos?

///////////////////////////////////////////////////////////////////////////
/// @brief Finds the number of vertices on the shortest path
/// from a source vertex to a goal vertex in an input graph. The function
/// uses Dijkstra's SSSP algorithm.
/// @param g The input graph.
/// @param wmap The weight-map to be used.
/// @param source The vertex descriptor for the source vertex.
/// @param goal The vertex descriptor for the goal vertex.
/// @param v A vector to hold the vertex descriptors of the path from source
/// to goal.
/// @param weight_max A max weight to be used in the SSSP algorithm.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph, typename EdgeMap>
size_t find_path_dijkstra(Graph& g,
                          EdgeMap& wmap,
                          typename Graph::vertex_descriptor source,
                          typename Graph::vertex_descriptor goal,
                          std::vector<typename Graph::vertex_descriptor>& v,
                          typename EdgeMap::property_value_type  weight_max
                          = std::numeric_limits<
                              typename EdgeMap::property_value_type>::max() )
{
  typedef typename Graph::vertex_descriptor VD;
  typedef map_property_map<Graph, VD>       pred_map_t;
  typedef map_property_map<Graph, typename EdgeMap::property_value_type>
                                                              dist_map_t;
  pred_map_t parentmap;
  dist_map_t distmap;
  dijkstra_sssp(g, parentmap, distmap, wmap, source, goal, weight_max);
  if (parentmap.get(goal) == goal)
    return 0;//there is no path
  //now prepare the return datastructure (the path)
  VD temp = goal;
  //push the last element
  v.push_back(goal);
  while (parentmap.get(temp) != source){
    v.push_back(parentmap.get(temp));
    temp = parentmap.get(temp);
  }
  v.push_back(parentmap.get(temp));
  std::reverse(v.begin(), v.end());
  return v.size();
}

///////////////////////////////////////////////////////////////////////////
/// @brief Finds the number of vertices along the shortest path
/// from a source to a goal vertex, using Dijkstra's SSSP algorithm. No weight
/// map is provided, so a default edge property map is used.
/// @param g The input graph.
/// @param source The vertex descriptor for the source vertex.
/// @param goal The vertex descriptor for the goal vertex.
/// @param v The vector used to hold the vertex descriptors of the path from
/// source to goal.
/// @param weight_max The max weight to be used for the SSSP algorithm.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph>
size_t find_path_dijkstra(Graph& g,
                          typename Graph::vertex_descriptor source,
                          typename Graph::vertex_descriptor goal,
                          std::vector<typename Graph::vertex_descriptor>& v,
                          typename Graph::edge_property weight_max
                          = std::numeric_limits<
                              typename Graph::edge_property>::max())
{
 edge_property_map<Graph> wmap(g);
 return find_path_dijkstra(g, wmap, source, goal, v, weight_max);
}


///////////////////////////////////////////////////////////////////////////
/// @brief Finds the shortest path between a source and goal
/// vertex in an input graph, using Dijkstra's SSSP algorithm. This function
/// not only finds the path from source to goal, but also the vertex properties
/// of all of the vertices along the path, as well as the edge properties of
/// all edges along the path.
/// @param g The input graph.
/// @param source The vertex descriptor for the source vertex.
/// @param goal The vertex descriptor for the goal vertex.
/// @param v The vector to hold the edge properties and vertex properties of
/// the path from source to goal.
/// @param weight_max The max weight used for the SSSP algorithm.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <typename Graph>
size_t find_path_dijkstra(Graph& g,
                          typename Graph::vertex_descriptor source,
                          typename Graph::vertex_descriptor goal,
                          std::vector<
                               std::pair<typename Graph::vertex_property,
                                         typename Graph::edge_property> >& v,
                          typename Graph::edge_property weight_max
                          = std::numeric_limits<
                              typename Graph::edge_property>::max())
{
  typedef typename Graph::vertex_descriptor VD;
  std::vector<VD> spath;
  size_t res = find_path_dijkstra(g,source, goal, spath, weight_max);
  if (res > 0){
    //convert vd to property
    typename Graph::vertex_property vp;
    typename Graph::edge_property ep;
    for (size_t i=0;i<spath.size()-1;++i){
      vp = get_vertex_data(g,spath[i]);
      ep = get_edge_data(g,spath[i],spath[i+1]);
      v.push_back(std::pair<typename Graph::vertex_property,
                            typename Graph::edge_property>(vp,ep));
    }
    vp = get_vertex_data(g,spath[spath.size()-1]);
    v.push_back(std::pair<typename Graph::vertex_property,
                          typename Graph::edge_property>(vp,ep));
  }
  return res;
}


}//namespace sequential
}//namespace stapl

#endif
