/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_BREADTH_FIRST_SEARCH_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_BREADTH_FIRST_SEARCH_HPP

#include <boost/pending/queue.hpp>
#include <queue>
#include "graph_algo_util.h"

namespace stapl{
namespace sequential{

//////////////////////////////////////////////////////////////////////
/// @brief Performs a Breadth-First Search (BFS) on
/// an input graph. A visitor class is used to visit each vertex reached by
/// the BFS.
/// @param g The graph to perform the BFS on.
/// @param s The vertex descriptor for the starting vertex.
/// @param vis The visitor class that will be used when visiting each vertex.
/// @param color_map The color map which stores a color for each vertex in
/// the graph.
/// @param Q The queue used to hold the vertices.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph, class BFSVisitor,class ColorMap, class QueueType>
void breadth_first_search1 (Graph& g,
                            typename Graph::vertex_descriptor s,
                            BFSVisitor& vis,
                            ColorMap& color_map,
                            QueueType& Q)
{

  typedef typename Graph::vertex_descriptor  vertex_descriptor;
  typedef typename Graph::vertex_iterator    vertex_iterator;

  typedef typename ColorMap::property_value_type color_value;
  typedef graph_color<color_value>               color_set;

  vertex_iterator vi_u, vi_v;
  typename Graph::adj_edge_iterator ei, ei_end;
  vi_u = g.find_vertex(s);
  color_map.put(*vi_u, color_set::gray());
  vis.discover_vertex(vi_u);
  Q.push(s);
  while (! Q.empty()) {
    vi_u = g.find_vertex(Q.front());
    Q.pop();
    vis.examine_vertex(vi_u);
    ei= (*vi_u).begin();
    ei_end = (*vi_u).end();
    for (; ei != ei_end; ++ei) {
      const vertex_descriptor& v = (*ei).target();
      vis.examine_edge(vi_u,ei);
      vi_v = g.find_vertex(v);
      const color_value& v_color = color_map.get(*vi_v);
      if (v_color == color_set::white()) {
        vis.tree_edge(vi_u,ei);
        color_map.put(*vi_v, color_set::gray());
        vis.discover_vertex(vi_v);
        Q.push(v);
      } else {
        vis.non_tree_edge(vi_u,ei);
        if (v_color == color_set::gray())
          vis.gray_target(vi_u,ei);
        else
          vis.black_target(vi_u,ei);
      }
    } // end for
    color_map.put(*vi_u, color_set::black());
    vis.finish_vertex(vi_u);
  } // end while
} // breadth_first_visit

//////////////////////////////////////////////////////////////////////
/// @brief Performs a breadth first search on the input
/// graph. No queue or queue type is specified, and a boost::queue is used.
/// @param g The input graph to perform BFS on.
/// @param s The starting vertex for the BFS.
/// @param vis The visitor class used to visit each vertex.
/// @param color_map The color map to be used.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph, class BFSVisitor,class ColorMap>
void breadth_first_search (Graph& g,
                           typename Graph::vertex_descriptor s,
                           BFSVisitor& vis,
                           ColorMap& color_map)
{
  boost::queue<typename Graph::vertex_descriptor> Q;
  stapl::sequential::breadth_first_search1(g, s, vis, color_map, Q);
}

//////////////////////////////////////////////////////////////////////
/// @brief Performs a BFS on a graph. No starting vertex
/// is given. Instead, BFS is run with different starting vertices until all
/// vertices in the graph have been reached by some BFS.
/// @param g Input graph to perform BFS on.
/// @param vis The visitor class used to visit each vertex.
/// @param color_map The color map to be used.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph, class BFSVisitor,class ColorMap>
void breadth_first_search (Graph&  g,
                           BFSVisitor& vis,
                           ColorMap& color_map)
{
  typedef typename ColorMap::property_value_type  color_value;
  typedef graph_color<color_value>                color_set;

  for (typename Graph::vertex_iterator vi = g.begin(); vi != g.end(); ++vi) {
    if (color_map.get(*vi) == color_set::white())
      breadth_first_search(g, (*vi).descriptor(), vis, color_map);
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief A BFS traversal where specific actions within visitors can stop the
/// traversal. This is useful for situations where we search for
/// something and we stop the traversal the moment we find it. The function
/// returns the EARLY_QUIT value if it is stopped early, and a CONTINUE
/// value otherwise.
/// @param g The input graph to perform the BFS on.
/// @param s The starting vertex for the BFS.
/// @param vis The visitor class used to visit each vertex. This visitor class
/// should have functions defined to return the EARLY_QUIT value when the
/// traversal should be stopped early.
/// @param color_map The color map which defines a color for each vertex
/// in the graph.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph, class BFSVisitor,class ColorMap>
visitor_return breadth_first_search_early_quit (Graph& g,
                                      typename Graph::vertex_descriptor s,
                                      BFSVisitor& vis,
                                      ColorMap& color_map)
{

  typedef typename Graph::vertex_descriptor vertex_descriptor;
  typedef typename Graph::vertex_iterator   vertex_iterator;
  typedef typename ColorMap::property_value_type  color_value;
  typedef graph_color<color_value>                color_set;

  std::deque<typename Graph::vertex_iterator> Q;
  vertex_iterator vi_u, vi_v;
  typename Graph::adj_edge_iterator ei, ei_end;
  vi_u = g.find_vertex(s);
  color_map.put(*vi_u, color_set::gray());
  if (vis.discover_vertex(vi_u)==EARLY_QUIT)
    return EARLY_QUIT;
  Q.push_back(vi_u);
  while (! Q.empty()) {
    vi_u = Q.front();
    Q.pop_front();
    if (vis.examine_vertex(vi_u)==EARLY_QUIT)
      return EARLY_QUIT;
    for (ei= (*vi_u).begin(); ei != (*vi_u).end(); ++ei) {
      vertex_descriptor v = (*ei).target();
      if (vis.examine_edge(vi_u,ei)==EARLY_QUIT)
        return EARLY_QUIT;
      vi_v = g.find_vertex(v);
      color_value v_color = color_map.get(*vi_v);
      if (v_color == color_set::white()) {
        if (vis.tree_edge(vi_u,ei)==EARLY_QUIT)
          return EARLY_QUIT;
        color_map.put(*vi_v, color_set::gray());
        if (vis.discover_vertex(vi_v)==EARLY_QUIT)
          return EARLY_QUIT;
        Q.push_back(vi_v);
      } else {
        if (vis.non_tree_edge(vi_u,ei)==EARLY_QUIT)
          return EARLY_QUIT;
        if (v_color == color_set::gray()){
          if (vis.gray_target(vi_u,ei)==EARLY_QUIT)
            return EARLY_QUIT;
        }
        else{
          if (vis.black_target(vi_u,ei)==EARLY_QUIT)
            return EARLY_QUIT;
        }
      }
    } // end for
    color_map.put(*vi_u, color_set::black());
    if (vis.finish_vertex(vi_u)==EARLY_QUIT)
      return EARLY_QUIT;
  } // end while
  return CONTINUE;
} // breadth_first_visit

//////////////////////////////////////////////////////////////////////
/// @brief BFS Traversal where specific actions within visitors can stop the
/// traversal. This is useful for situations where we search for
/// something and we stop the traversal the moment we find it. This function
/// does not take a starting vertex parameter.Instead, BFS is run with
/// different starting vertices until all vertices in the graph have been
/// reached by some BFS.
/// @param g The input graph to perform the BFS on.
/// @param vis The visitor class used to visit each vertex. This visitor class
/// should have functions defined to return the EARLY_QUIT value when the
/// traversal should be stopped early.
/// @param color_map The color map which defines a color for each vertex
/// in the graph.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph, class BFSVisitor,class ColorMap>
visitor_return breadth_first_search_early_quit (Graph&  g,
                           BFSVisitor& vis,
                           ColorMap& color_map)
{
  typedef typename ColorMap::property_value_type  color_value;
  typedef graph_color<color_value>                color_set;

  for (typename Graph::vertex_iterator vi = g.begin(); vi != g.end(); ++vi) {
    if (color_map.get(*vi) == color_set::white()) {
      if (breadth_first_search_early_quit(g, (*vi).descriptor(), vis, color_map)
          == EARLY_QUIT)
        return EARLY_QUIT;
    }
  }
  return CONTINUE;
}


}//namespace sequential
}//namespace stapl

#endif
