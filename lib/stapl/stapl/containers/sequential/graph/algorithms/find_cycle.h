/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_FIND_CYCLE_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_FIND_CYCLE_HPP

#include <deque>
#include "depth_first_search.h"
#include "breadth_first_search.h"

namespace stapl{
namespace sequential{

//////////////////////////////////////////////////////////////////////
/// @brief Checks for cycles in a directed graph. It does so
/// by returning EARLY_QUIT if a visitor is visiting a node that has
/// already been visited.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
//////////////////////////////////////////////////////////////////////
template <class GRAPH>
class visitor_cycle
 : public visitor_base<GRAPH>
{
  //////////////////////////////////////////////////////////////////////
  /// @brief The visitor class function for a gray target, which will
  /// return EARLY_QUIT.
  //////////////////////////////////////////////////////////////////////
  public:
  visitor_return gray_target (typename GRAPH::vertex_iterator ,
                              typename GRAPH::adj_edge_iterator )
  {
    return EARLY_QUIT;
  }
  /*
  visitor_return black_target (typename GRAPH::vertex_iterator ,
                               typename GRAPH::adj_edge_iterator ){
    return EARLY_QUIT;
  }
  */
};

//////////////////////////////////////////////////////////////////////
/// @brief Checks for cycles in a directed graph by running a
/// Depth-First Search with the visitor_cycle class as the visitor class.
/// @param _g The input graph.
/// @param _color_map A colormap for the depth first search, which maps
/// each vertex in the graph to a color.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template<class Graph, class ColorMap>
bool is_cycle (Graph& _g, ColorMap& _color_map)
{
  visitor_cycle<Graph> vis;
  visitor_return res = depth_first_search_early_quit(_g, vis, _color_map);
  if (res == EARLY_QUIT) return true;
  else return false;
}

//////////////////////////////////////////////////////////////////////
/// @brief Finds the back edges in a graph.
/// @tparam GRAPH The type of input graph.
/// @tparam Container The type of container to hold the back edges.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
//////////////////////////////////////////////////////////////////////
 template <class GRAPH, class Container>
class visitor_back_edges
 : public visitor_base<GRAPH>
{
  Container& m_back_edges;
  public:

  visitor_back_edges(Container& _v)
   : m_back_edges(_v){};

  //////////////////////////////////////////////////////////////////////
  /// @brief Used for a black target, which pushes
  /// a back edge into the container.
  /// @param vi The vertex iterator.
  /// @param ei The edge iterator.
  //////////////////////////////////////////////////////////////////////
  visitor_return black_target (typename GRAPH::vertex_iterator vi,
                                      typename GRAPH::adj_edge_iterator ei)
  {
    typedef typename GRAPH::vertex_descriptor VD;
    m_back_edges.push_back(std::pair<VD,VD>((*ei).target(),(*vi).descriptor()));
    return CONTINUE;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Uses a Breadth-First Search with the
/// visitor_back_edges visitor class to find all of the back edges in
/// and input graph. Return is the number of back edges.
/// @param _g The input graph.
/// @param _color_map The color map to be used, which maps each vertex in
/// the graph to a color.
/// @param _V A container to hold the back edges of the graph.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template<class Graph, class ColorMap, class Container>
size_t get_back_edges(Graph& _g, ColorMap& _color_map, Container& _V)
{
   visitor_back_edges<Graph,Container> vis(_V);
   breadth_first_search(_g , vis, _color_map);
   return _V.size();
}

}//namespace sequential
}//namespace stapl

#endif
