/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_ALGO_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_ALGO_HPP

#include <deque>
#include "breadth_first_search.h"
#include "depth_first_search.h"

namespace stapl{
namespace sequential{


// CLIQUE

//////////////////////////////////////////////////////////////////////
/// @brief Checks if a subgraph is fully connected.
/// @param g The input graph.
/// @param vids A vector containing the vertices of the subgraph.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class VGraph>
bool is_clique(const VGraph& g,
               std::vector<typename VGraph::vertex_descriptor>& vids)
{
  typename VGraph::const_vertex_iterator   vi;
  typename VGraph::const_adj_edge_iterator ei;
  size_t sz = vids.size();
  for (size_t i=0; i<sz; i++){
    for (size_t j=0; j<sz; j++){
      typename VGraph::edge_descriptor ed(vids[i],vids[j]);
      if ((i!=j) && (!g.find_edge(ed, vi, ei)) )
        return false;
    }
  }
  return true;
}

//////////////////////////////////////////////////////////////////////
/// @brief Checks if a subgraph formed by the specified
/// vertex and its adjacent vertices is fully connected.
/// @param g The input graph.
/// @param vd The vertex descriptor of the desired vertex.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class VGraph>
bool is_clique(const VGraph& g, typename VGraph::vertex_descriptor vd)
{
  std::vector<typename VGraph::vertex_descriptor> vd_vec;
  //g.get_adjacent_vertices(vd, vd_vec); // this assumes a more restrictive view
  typename VGraph::const_vertex_iterator   vi;
  typename VGraph::const_adj_edge_iterator ei;
  vi = g.find_vertex(vd);
  if (vi==g.end()) return false;
  else {
    vd_vec.reserve((*vi).size());
    for (ei=(*vi).begin();ei!=(*vi).end();++ei){
      vd_vec.push_back((*ei).target());
    }
  }
  return is_clique(g, vd_vec);
}

//////////////////////////////////////////////////////////////////////
/// @brief Takes a directed input graph and creates a copy of the
/// graph with reversed edges.
/// @param _og The original graph.
/// @param _rg A reference to the graph object where the reverse graph
/// will be stored.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class ReverseG, class OriginalG>
void build_reverse_graph(OriginalG& _og, ReverseG& _rg)
{
  typename OriginalG::vertex_iterator vi;
  typedef typename ReverseG::vertex_property VP;
  VP p = VP();
  for (vi=_og.begin();vi!=_og.end();++vi){
    typename ReverseG::vertex_descriptor vd = ((size_t)(*vi).descriptor());
    _rg.add_vertex(vd,p);
  }

  for (vi=_og.begin();vi!=_og.end();++vi){
    typename OriginalG::adj_edge_iterator ei = (*vi).begin();
    while (ei != (*vi).end()){
      _rg.add_edge((size_t)(*ei).target(), (size_t)(*ei).source());
      ++ei;
    }
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief Creates a directed graph with predecessors based
/// on an input graph.
/// @param _og The input graph.
/// @param _rg A reference to the graph object where the predecessors graph
/// will be stored.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class DPG, class OriginalG>
void build_directed_preds_graph(OriginalG& _og, DPG& _rg)
{
  typename OriginalG::vertex_iterator vi;
  typename DPG::vertex_property p;
  for (vi=_og.begin();vi!=_og.end();++vi){
    typename DPG::vertex_descriptor vd = ((size_t)(*vi).descriptor());
    _rg.add_vertex(vd,p);
  }
  _rg.set_lazy_update(true);
  for (vi=_og.begin();vi!=_og.end();++vi){
    typename OriginalG::adj_edge_iterator ei = (*vi).begin();
    while (ei != (*vi).end()){
      _rg.add_edge((size_t)(*ei).source(),(size_t)(*ei).target());
      ++ei;
    }
  }
  _rg.set_lazy_update(false);
}

//////////////////////////////////////////////////////////////////////
/// @brief Finds the number of sinks (vertices with out-degree
/// of zero) in a graph.
/// @param g The input graph.
/// @param _sinks A vector to hold the vertex descriptors of the sinks.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class VGraph>
size_t get_sinks(const VGraph& g,
                 std::vector<typename VGraph::vertex_descriptor>& _sinks)
{
  _sinks.clear();
  for (typename VGraph::const_vertex_iterator cvi=g.begin();cvi!=g.end();++cvi){
    if ((*cvi).size() == 0){
      _sinks.push_back((*cvi).descriptor());
    }
  }
  return _sinks.size();
}

}//namespace sequential
}//namespace stapl

#endif
