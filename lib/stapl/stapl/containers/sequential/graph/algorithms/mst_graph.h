/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_MST_GRAPH_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_MST_GRAPH_HPP

#include "values.h"
#include "util/d_ary_heap.h"
#include "connected_components.h"
#include <algorithm>
#include <vector>

#include <boost/config.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/pending/mutable_queue.hpp>
#include <boost/pending/relaxed_heap.hpp>


namespace stapl{
namespace sequential{

// helper functions and classes

// CAUTION:: Swap the order of dv1 and dv2 when switching from
//           std::min_element() to heap!!!

//////////////////////////////////////////////////////////////////////
/// @brief Takes two vertex descriptors and compares
/// their entries in a map.
/// @param comp_ The comparator to be used.
/// @param map_ The map to be used.
/// @param dv1 The first vertex descriptor.
/// @param dv2 The second vertex descriptor.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
//////////////////////////////////////////////////////////////////////
template<typename VD, typename Comparator, typename Map>
struct wfunc_comp
{
  Comparator comp;
  Map* map;
  wfunc_comp(Comparator& comp_, Map* map_)
   : comp(comp_), map(map_)
   {
   }

  bool operator()(const VD& dv1, const VD& dv2) const
  {
    return (comp(map->get(dv1), map->get(dv2)));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compares the properties of two edges.
/// @param comp The comparator to be used.
/// @param ei1 The first edge iterator.
/// @param ei2 The second edge iterator.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
//////////////////////////////////////////////////////////////////////
template<typename EI, typename Comparator>
struct eiwfunc_comp
{
  Comparator comp;
  eiwfunc_comp( Comparator& comp_)
   : comp(comp_)
   {
   }

  bool operator()(EI ei1, EI ei2)
  {
    return (comp((*ei1).property(),
                 (*ei2).property()));
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Finds the cheapest edge from a vertex to another
/// component of the graph.
/// @param g The input graph.
/// @param vd The vertex descriptor.
/// @param vi The vertex iterator for the graph.
/// @param ei_junk The edge iterator.
/// @param verts_in_same_cc A vector containing the vertices that are in the
/// same connected component as vd.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
//////////////////////////////////////////////////////////////////////
template <class G, class VD, class VI, class EI>
EI cheapest_edge_to_different_component(G& g, VD vd, VI vi, EI ei_junk,
                                        std::vector<VD>& verts_in_same_cc)
{
  typedef vector_property_map<G,size_t> cmap_type;

  EI cheapest((*vi).begin());
  for (EI it = (*vi).begin(); it != (*vi).end(); ++it) {
    if ((*it).property() < (*cheapest).property()) {
            if (find(verts_in_same_cc.begin(), verts_in_same_cc.end(),
               (*it).target()) == verts_in_same_cc.end())
               // !is_same_cc(g, cmap, (*it).source(), (*it).target()))
              cheapest = it;
        }
    }
    cmap_type cmap;
    if (is_same_cc(g, cmap, (*cheapest).source(), (*cheapest).target()))
      cheapest = (*vi).end();
    return cheapest;
}

//////////////////////////////////////////////////////////////////////
/// @brief Finds the minimum element in a vector of edge
/// iterators.
/// @param S The vector containing the edge iterators.
/// @param comp The comparator used to compare two edge properties.
/// @todo This function should be removed and its use should be replaced
/// with a call to std::min_element.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
//////////////////////////////////////////////////////////////////////
template <class EI, class Comparator>
EI min_element(std::vector<EI>& S, Comparator& comp)
{
  EI curr(*(S.begin()));
  for (typename std::vector<EI>::iterator it = S.begin(); it != S.end(); ++it){
    if ((*(*it)).property() < (*curr).property()) {
      curr = *it;
    }
  }
  return curr;
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds a Minimum Spanning Tree (MST) of a graph,
/// using sorted edges and Kruskal's algorithm.
/// @param g The weighted, undirected input graph.
/// @param mst A reference to the graph object that will hold the MST.
/// @param source A source vertex for the algorithm.
/// @param comp A comparator to compare and sort the edges.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph, class MST_Graph, typename Comparator>
void mst_kruskals(Graph& g, MST_Graph& mst,
               typename Graph::vertex_descriptor source,
               Comparator& comp)
{

  typedef typename Graph::vertex_iterator VI;
  typedef typename Graph::edge_iterator EI;

  typedef vector_property_map<Graph,size_t> cmap_type;
  typedef eiwfunc_comp<EI, Comparator> ei_weight_comp;

  std::vector<EI> Edges;

  // MST = g.vertices();
  mst.clear();
  for (VI vi = g.begin(); vi != g.end(); ++vi) {
    mst.add_vertex((*vi).descriptor());
  }

  for (EI ei = g.edges_begin(); ei != g.edges_end(); ++ei) {
    Edges.push_back(ei);
  }

  std::sort(Edges.begin(), Edges.end(), ei_weight_comp(comp));

  // WARNING: NEED TO TAKE CARE OF NON-CONNECTED GRAPHS!!!
  // for each sorted edges
  for (size_t i = 0; i < Edges.size(); ++i) {
    EI ei = Edges[i];  // pick the lowest edge_wt.
    cmap_type cmap;
    // if vertices connected by the edge are in different CCs, add the edge.
    if (!is_same_cc(mst, cmap, (*ei).source(), (*ei).target())) {
    // does a BFS on G.
      // add the edge.
      mst.add_edge((*ei).descriptor(), (*ei).property());
      if (mst.is_directed()) { // for Directed G, to maintain CCs.
        mst.add_edge(reverse((*ei).descriptor()), (*ei).property());
      }
    }
  }
}


// Boruvka's/Sollin's Algorithm -- good for parallelization.
// Runtime: O(E*logV) for sequential.

//////////////////////////////////////////////////////////////////////
/// @brief Finds a Minimum Spanning Tree (MST) of a graph,
/// using Boruvka's/Sollin's algorithm.
/// @param g The weighted, undirected input graph.
/// @param mst A reference to the graph object that will hold the MST.
/// @param source A source vertex for the algorithm.
/// @param comp A comparator to compare and sort the edges.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph, class MST_Graph, typename Comparator>
void mst_sollins(Graph& g, MST_Graph& mst,
                 typename Graph::vertex_descriptor source,
                 Comparator& comp)
{

  // #define INVALID_VD -1
  typedef typename Graph::vertex_iterator VI;
  typedef typename Graph::vertex_descriptor VD;
  typedef typename Graph::adj_edge_iterator EI;

  typedef vector_property_map<Graph,size_t> cmap_type;

  // WARNING: WILL MODIFY ORIGINAL EDGEWT IN INPUT GRAPH!!!
  EI ei_max((*g.begin()).begin());

  // MST = g.vertices();
  mst.clear();
  for (VI vi = g.begin(); vi != g.end(); ++vi) {
    mst.add_vertex((*vi).descriptor());
  }

  // WARNING: NEED TO TAKE CARE OF NON-CONNECTED GRAPHS!!!
  // While(vertices of MST connected by T are disjoint)
  size_t num_cc = mst.get_num_vertices();

  while (num_cc > 1) {
    cmap_type cmap;
    std::vector< std::pair<size_t,VD> > _ccstats;

    get_cc_stats (mst, cmap, _ccstats);

    std::vector<VD> invalid_vec;
    // For_each(component in MST)
    typename std::vector< std::pair<size_t,VD> >::iterator it;

    for (it = _ccstats.begin(); it != _ccstats.end(); ++it) {
      if (find(invalid_vec.begin(), invalid_vec.end(), (*it).second)
          != invalid_vec.end()) {
        continue;
      }
      std::vector<EI> S;
      // For_each(vertex in component)
      std::vector<VD> verts_in_same_cc;
      cmap.reset();

      get_cc(mst, cmap, (*it).second, verts_in_same_cc);

      for (size_t j = 0; j < verts_in_same_cc.size(); ++j) {
        VI vi = g.find_vertex(verts_in_same_cc[j]);
        // Add the cheapest edge from the vertex in the component
        EI min = cheapest_edge_to_different_component(mst, (*it).second, vi,
                                                   ei_max, verts_in_same_cc);
        if (!(min == (*vi).end()))
          S.push_back(min);  // to another vertex in a disjoint component to S
      }

      if (S.size() == 0) {
        cmap.reset();
        num_cc = get_cc_count(mst, cmap);
        continue;
      }
      EI min_edge = min_element(S, comp);
      if (!mst.is_edge((*min_edge).source(), (*min_edge).target())
          && !mst.is_edge((*min_edge).target(), (*min_edge).source())) {
        mst.add_edge((*min_edge).descriptor(), (*min_edge).property());
        if (mst.is_directed()) { // for Directed G, to maintain CCs.
          mst.add_edge(reverse((*min_edge).descriptor()),
                                 (*min_edge).property());
        }
        --num_cc;
        invalid_vec.push_back((*min_edge).target());
      }
    }
  }
}



// Prim's Algorithm -- good for sequential.
// Runtime: O(E*logV) for sequential.
// @g should be a weighted, undirected graph with 'non-negative' edge weights.

//////////////////////////////////////////////////////////////////////
/// @brief Finds a Minimum Spanning Tree (MST) of a graph,
/// using Prim's algorithm.
/// @param g The weighted, undirected input graph.
/// @param parentmap A map to hold the parent of each vertex.
/// @param distmap A map to hold the distance from each vertex to the source.
/// @param weightmap A map to hold the edge weights.
/// @param source A source vertex for the algorithm.
/// @param comp A comparator to compare and sort the edges.
/// @param colormap A map that maps each vertex in the graph to a color.
/// @param Q A queue to hold the vertices during the algorithm.
/// @todo This updated version of Prim's algorithm is untested.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph, typename Comparator, class PMap,
          class DMap, class WMap, class CMap, class Queue>
void mst_prims(Graph& g,
               PMap& parentmap,
               DMap& distmap,
               WMap& weightmap,
               typename Graph::vertex_descriptor source,
               Comparator& comp,
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
    VR u = Q.top(); Q.pop();
    AEI ei_end = u.end();
    AEI ei = u.begin();

    for (; ei!=ei_end; ei++) {
      VD v = (*ei).target();
      color_type color = colormap.get(v);
      if (color == color_set::white()) {
        //discovered a new vertex; relax and push into the queue.
        VI v_iter = g.find_vertex(v);
        dist_type d_u, d_v;
        d_u = distmap.get(u); d_v = distmap.get(*v_iter);
        const Weight& weight = weightmap.get(*ei);
        if (comp(weight, d_v)) {
          distmap.put(*v_iter,weight);
          parentmap.put(*v_iter, u.descriptor());
        }
        colormap.put(v, color_set::gray());
        Q.push(*v_iter);
      }
      else if (color == color_set::gray()) {
        //vertex is already in the queue; relax and update if necessary.
        VI v_iter = g.find_vertex(v);
        dist_type d_u, d_v;
        d_u = distmap.get(u); d_v = distmap.get(*v_iter);
        const Weight& weight = weightmap.get(*ei);
        if (comp(weight, d_v)) {
          distmap.put(*v_iter, weight);
          parentmap.put(*v_iter, u.descriptor());
          if (comp(distmap.get(*v_iter), d_v))
            Q.update(*v_iter);
        }
      }
    }
    colormap.put(u, color_set::black());
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief Finds a Minimum Spanning Tree (MST) of a graph,
/// using Prim's algorithm. No queue is given, so a default queue is used.
/// No colormap is provided, so a default color map with all vertices
/// initialized to white is used.
/// @param g The weighted, undirected input graph.
/// @param parentmap A map to hold the parent of each vertex.
/// @param distmap A map to hold the distance from each vertex to the source.
/// @param weightmap A map to hold the edge weights.
/// @param source A source vertex for the algorithm.
/// @param comp A comparator to compare and sort the edges.
/// @param weight_max A max weight to used in the algorithm.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph, class PMap, class DMap, class WMap, typename Comparator>
void mst_prims(Graph& g,
               PMap& parentmap,
               DMap& distmap,
               WMap& weightmap,
               typename Graph::vertex_descriptor source,
               Comparator& comp,
               typename WMap::property_value_type weight_max
               = std::numeric_limits<
                   typename WMap::property_value_type>::max())
{
  typedef typename Graph::vertex_iterator    VI;
  typedef typename Graph::vertex_reference   VR;
  typedef typename WMap::property_value_type Weight;
  typedef map_property_map<Graph, size_t>    index_map_t;

  map_property_map<Graph, size_t> colormap;
  index_map_t index_map;

  d_ary_heap_indirect<VR, 4, index_map_t, DMap, Comparator>
    queue(distmap, index_map, comp);

  VI vi = g.begin(); VI vi_end = g.end();
  for (; vi != vi_end; ++vi) {
    VR v = *vi;
    distmap.put(v, weight_max);
    parentmap.put(v, v.descriptor());
    colormap.put(v, graph_color<size_t>::white());
  }
  distmap.put(source, Weight());

  mst_prims(g, parentmap, distmap, weightmap,
            source, comp, colormap, queue);
}

//////////////////////////////////////////////////////////////////////
/// @brief Finds a Minimum Spanning Tree (MST) of a graph,
/// using Prim's algorithm. No queue is given, so a default queue is used.
/// No colormap is provided, so a default color map with all vertices
/// initialized to white is used. No comparator is provided, so a less than
/// comparator is used.
/// @param g The weighted, undirected input graph.
/// @param parentmap A map to hold the parent of each vertex.
/// @param distmap A map to hold the distance from each vertex to the source.
/// @param weightmap A map to hold the edge weights.
/// @param source A source vertex for the algorithm.
/// @param weight_max A max weight to be used in the algorithm.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph, class PMap, class DMap, class WMap>
void mst_prims(Graph& g,
               PMap& parentmap,
               DMap& distmap,
               WMap& weightmap,
               typename Graph::vertex_descriptor source,
               typename Graph::edge_property weight_max
               = std::numeric_limits<typename Graph::edge_property>::max())
{
  typedef typename WMap::property_value_type Weight;
  std::less<Weight> lsop;
  mst_prims(g, parentmap, distmap, weightmap, source, lsop, weight_max);
}

//////////////////////////////////////////////////////////////////////
/// @brief Finds a Minimum Spanning Tree (MST) of a graph,
/// using Boruvka's/Sollin's algorithm. No comparator is given, so the
/// less than comparator is used.
/// @param g The weighted, undirected input graph.
/// @param mst A reference to the graph object that will hold the MST.
/// @param source A source vertex for the algorithm.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph, class MST_Graph>
void mst_sollins(Graph& g, MST_Graph& mst,
                 typename Graph::vertex_descriptor source)
{
  typedef typename Graph::edge_property Weight;
  std::less<Weight> lsop;
  mst_sollins(g, mst, source,lsop);
}

//////////////////////////////////////////////////////////////////////
/// @brief Finds a Minimum Spanning Tree (MST) of a graph,
/// using sorted edges and Kruskal's algorithm. No comparator is given,
/// so the less than comparator is used.
/// @param g The weighted, undirected input graph.
/// @param mst A reference to the graph object that will hold the MST.
/// @param source A source vertex for the algorithm.
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Graph, class MST_Graph>
void mst_kruskals(Graph& g, MST_Graph& mst,
               typename Graph::vertex_descriptor source)
{
  typedef typename Graph::edge_property Weight;
  std::less<Weight> lsop;
  mst_kruskals(g, mst, source,lsop);
}

}//namespace sequential
}//namespace stapl

#endif
