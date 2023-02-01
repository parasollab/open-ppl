/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_COUNT_HOP_PAIRS_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_COUNT_HOP_PAIRS_HPP

#include "graph_algo_util.h"
#include "breadth_first_search.h"

namespace stapl {
namespace sequential{

namespace hops_detail {

////////////////////////////////////////////////////////////////////////////////
/// @brief Find all of the vertices that are no more than @p m_limit hops
/// away from the source vertex.
/// @tparam Graph the graph being traversed
/// @note Visitor for use with BFS-early-quit.
/// @ingroup seqGraphUtil
////////////////////////////////////////////////////////////////////////////////
template<typename Graph>
class hops_visitor
        : public visitor_base<Graph>
{
  typedef typename Graph::vertex_iterator   vertex_iterator;
  typedef typename Graph::adj_edge_iterator edge_iterator;
  typedef typename Graph::vertex_descriptor vertex_descriptor;

  typedef visitor_base<Graph>             base_type;
  typedef map_property_map<Graph, size_t> map_type;

  /// the graph
  Graph*    m_graph;
  /// the map of the property value for each vertex
  map_type* m_map;
  /// the maximum number of hops allowed
  size_t    m_limit;
  /// the number of vertices within the given range
  size_t    m_count;
  /// the vector that stores the selected vertices
  vector <vertex_descriptor>& m_vec;

public:
  hops_visitor(Graph& _g, map_type& _map,
               size_t _limit, vector<vertex_descriptor>& _vec)
    : base_type(_g), m_graph(&_g), m_map(&_map),
      m_limit(_limit), m_count(0), m_vec(_vec)
  { }

  visitor_return tree_edge(vertex_iterator vi, edge_iterator ei)
  {
    //when tree edge is called, we've found a new vertex; add one to
    //the source's depth and assign that as the target's depth. If
    //we've reached the limit, quit instead.
    size_t depth = m_map->get(*vi) + 1;
    if (depth <= m_limit) {
      m_map->put((*ei).target(), depth);
      m_vec.push_back((*ei).target());
      ++m_count;
      return CONTINUE;
    }
    else return EARLY_QUIT;
  }
  size_t get_count() {return m_count;}
};

}//namespace hops_detail

/////////////////////////////////////////////////////////////////////////
/// @brief Given @p v, count the number of vertex pairs where the shortest
/// path from @p v to another vertex in terms of number of edges is at most
/// @p num_hops.
/// @param g the graph
/// @param num_hops the maximum number of hops allowed
/// @param v the source vertex for the search
/// @return the number of pairs found
/// @note The complexity for this search  is O(|V|+|E|).
/// @ingroup seqGraphAlgo
/////////////////////////////////////////////////////////////////////////
template<typename Graph>
size_t count_hop_pairs(Graph& g, size_t num_hops,
                       typename Graph::vertex_descriptor v)
{
  //hopmap: VD -> number of edges in a shortest-path traversal from v to u.
  map_property_map<Graph, size_t> hopmap;
  hopmap.put(v, 0);

  map_property_map<Graph, size_t> colormap;
  vector <typename Graph::vertex_descriptor> x;
  hops_detail::hops_visitor<Graph> vis(g,hopmap,num_hops,x );
  //early quits when num_hops distance from v is reached
  breadth_first_search_early_quit(g,v,vis,colormap);

  return vis.get_count();
}

///////////////////////////////////////////////////////////////////////////////
/// @brief Count the number of vertex pairs (u, v) in the graph where the
/// shortest path between u and v in terms of the number of edges is at most
/// @p num_hops.
/// @param g the graph
/// @param num_hops the maximum number of edges allowed in the search
/// @return the number of pairs found
/// @note The complexity for the search is O(|V|*(|V|+|E|)).
/// @ingroup seqGraphAlgo
////////////////////////////////////////////////////////////////////////////////
template<typename Graph>
size_t count_hop_pairs(Graph& g, size_t num_hops)
{
  size_t total = 0;
  typename Graph::vertex_iterator it, it_end = g.end();
  for (it = g.begin(); it != it_end; ++it)
    total += count_hop_pairs(g, num_hops, (*it).descriptor());

  //each valid vertex pair is seen exactly 2 times.
  return total/2;
}

}//namespace sequential
}//namespace stapl

#endif
