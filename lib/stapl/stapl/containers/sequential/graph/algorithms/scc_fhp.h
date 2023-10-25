/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_SCC_FHP_ALGO_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_SCC_FHP_ALGO_HPP

#include <deque>
#include "../graph.h"
#include "../directed_preds_graph.h"
#include "breadth_first_search.h"

namespace stapl{
namespace sequential{

///////////////////////////////////////////////////////////////////////
// SCC using Lisa K. Fleischer, Bruce Hendrickson, Ali Pinar version
///////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
/// @brief Object that stores the color and status properties.
/// @tparam Color the color type of the vertex
/// @ingroup seqGraphUtil
//////////////////////////////////////////////////////////////////////
template <class Color>
class internal_scc_property
{
  /// stores the value that represents the status of a vertex
  size_t m_status;//0 unmarked; 1 - marked dfs forward; 2 - backward; 3 both
  /// stores the color of the vertex
  Color  m_color;
 public:
  typedef Color color_type;
  internal_scc_property()
    : m_status(0)
  { }

  //status
  void set_status(size_t c)
  {
    m_status = c;
  }
  size_t get_status()
  {
    return m_status;
  }

  //color
  void set_color(Color c)
  {
    m_color = c;
  }
  Color get_color()
  {
    return m_color;
  }

  void reset()
  {
    m_status = 0;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Object that allows for manipulation of the color property
/// @tparam Property represents the color property
/// @ingroup seqGraphUtil
/// //////////////////////////////////////////////////////////////////
template <class Property>
class internal_scc_color_func
{
 public:
  typedef typename Property::color_type value_type;

  value_type get(Property& p)
  {
    return p.get_color();
  }

  void put(Property& p, value_type const& v)
  {
    p.set_color(v);
  }

  template <class Functor>
  void apply(Property& p, Functor f)
  {
    f(p);
  }
};

///////////////////////////////////////////////////////////////////////
/// @brief Takes a vertex and marks it as discovered while moving forward.
/// @tparam VGraph the graph view
/// @ingroup seqGraphAlgoWf
/// @todo move into a detail namespace
/////////////////////////////////////////////////////////////////////////
template <class VGraph>
class visitor_scc_fwd
  : public visitor_base<VGraph>
{
 public:
  visitor_return discover_vertex(typename VGraph::vertex_iterator vi)
  {
    (*vi).property().set_status(1);
    return CONTINUE;
  }
};

///////////////////////////////////////////////////////////////////////
/// @brief Takes a vertex and marks it as discovered while moving backward.
/// @tparam VGraph the graph view
/// @ingroup seqGraphAlgoWf
/// @todo move into a detail namespace
//////////////////////////////////////////////////////////////////////
template <class VGraph>
class visitor_scc_back
        : public visitor_predecessors_base<VGraph>
{
 public:
  visitor_return discover_vertex(typename VGraph::vertex_iterator vi)
  {
    size_t status = (*vi).property().get_status();
    if (status == 0)
      (*vi).property().set_status(2);
    else
      (*vi).property().set_status(3);
    return CONTINUE;
  }
};


// scc versions that deletes the vertices and edges of the input view/graph

///////////////////////////////////////////////////////////////////////
/// @brief Takes a graph and produces its strongly-connected components.
/// @param dpg the graph view
/// @param cmap the map of colors for the graph
/// @param scc the resulting container of strongly connected components,
/// likely a vector of vectors.
/// @note The vertices and edges of the input graph are deleted as the
/// algorithm executes.
/// @ingroup seqGraphAlgo
////////////////////////////////////////////////////////////////////////
template <class VGraph, class ColorMap, class OutputMap>
void scc_pinar_mutable(VGraph& dpg, ColorMap& cmap, OutputMap& scc)
{
  typedef typename VGraph::vertex_descriptor  VD;

  //vertices and edges to be removed;
  std::vector<VD>                                 vtor;
  std::vector<typename VGraph::edge_descriptor>   etor;

  visitor_scc_fwd<VGraph>  vis_fwd;
  visitor_scc_back<VGraph> vis_back;
  while (dpg.get_num_vertices() > 0){
    //pick a pivot
    VD pivot = (*dpg.begin()).descriptor();
    //do dfs forward
    depth_first_search(dpg,pivot,vis_fwd, cmap);
    cmap.reset();
    //do dfs backward
    depth_first_search_preds(dpg,pivot,vis_back,cmap);
    cmap.reset();

    //no mark and remove SCC; remove cross edges
    scc.push_back(std::vector<typename VGraph::vertex_descriptor>());
    for (typename VGraph::vertex_iterator vi=dpg.begin();vi != dpg.end();++vi){
      size_t status = (*vi).property().get_status();
      if ( status == 3) {
        scc.back().push_back((*vi).descriptor());
      }
      else {
        //v is not part of a scc
        //check all succ edges
        for (typename VGraph::adj_edge_iterator ei=(*vi).begin();
            ei != (*vi).end(); ++ei){
          typename VGraph::vertex_iterator vi2=dpg.find_vertex((*ei).target());
          if (status != (*vi2).property().get_status()){
            //mark the edge to be removed;
            etor.push_back((*ei).descriptor());
          }
        }//for all edges
      }//else
    }//for all vertices;

    //here delete cross edges
    for (size_t i =0 ;i<etor.size();++i){
      dpg.delete_edge(etor[i]);
    }

    //here delete scc vertices
    for (size_t i =0 ;i<scc.back().size();++i){
      dpg.delete_vertex(scc.back()[i]);
    }

    //reset the maps/ edge storage for new traversal
    for (typename VGraph::vertex_iterator vi=dpg.begin(); vi!=dpg.end();++vi){
      (*vi).property().reset();
    }
    etor.clear();
  }//end while
}//end scc_mutable

////////////////////////////////////////////////////////////////////
/// @brief Makes a copy of a graph and computes its strongly-connected
/// components.
/// @param g the graph
/// @tparam ColorMap the container of the graph's colors
/// @param scc the resulting container of strongly-connected components,
/// likely a vector of vectors.
/// @note The vertices and edges of the original graph are not deleted
/// in this algorithm.
/// @ingroup seqGraphAlgo
////////////////////////////////////////////////////////////////////
template <class VGraph, class ColorMap, class OutputMap>
void scc_pinar(VGraph& g,
               ColorMap&,
               OutputMap& scc)
{
  typedef typename ColorMap::property_value_type  color_type;
  typedef internal_scc_property<color_type>       vertex_property_type;
  typedef internal_scc_color_func<vertex_property_type> color_func_type;
  typedef directed_preds_graph<MULTIEDGES,vertex_property_type> dpg_type;
  typedef vertex_property_map<dpg_view<dpg_type>,color_func_type>
            color_map_type;

  //build a directed with predecessors graph;
  dpg_type dpg;
  build_directed_preds_graph(g, dpg);

  dpg_view<dpg_type> dp_view(dpg);

  color_map_type cmap(dp_view);
  typedef typename std::vector<
                     std::vector<typename dpg_view<dpg_type>::vertex_descriptor>
                   >
                     TempVecType;
  TempVecType scc2;
  cmap.reset();

  scc_pinar_mutable(dp_view, cmap, scc2);

  typename TempVecType::iterator it = scc2.begin();
  typename TempVecType::iterator it_end = scc2.end();
  for (; it != it_end; ++it) {
    typename OutputMap::value_type temp_vec;
    typename TempVecType::value_type::iterator iit = (*it).begin();
    typename TempVecType::value_type::iterator iit_end = (*it).end();
    for (; iit != iit_end; ++iit) {
      size_t temp = *iit;
      temp_vec.push_back(temp);
    }
    scc.push_back(temp_vec);
  }
}

}//namespace sequential
}//namespace stapl

#endif
