/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_DEPTH_FIRST_SEARCH_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_DEPTH_FIRST_SEARCH_HPP

#include <deque>
#include "graph_algo_util.h"

namespace stapl{
namespace sequential{

///////////////////////////////////////////////////////////////////////////
/// @brief Performs a Depth-First Search (DFS) on a given input
/// graph. A visitor class is used to visit each vertex reached by the DFS.
/// @param g The input graph to perform the DFS on.
/// @param s The starting vertex in the graph for the DFS.
/// @param vis The visitor class used to visit each vertex that is reached
/// during the DFS.
/// @param color_map The color map, which holds a color for each vertex in
/// the graph.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <class Graph, class DFSVisitor,class ColorMap>
void depth_first_search(Graph& g,
      typename Graph::vertex_descriptor s,
      DFSVisitor& vis,
      ColorMap& color_map)
{

  typedef typename Graph::vertex_descriptor  vertex_descriptor;
  typedef typename Graph::vertex_iterator    vertex_iterator;
  typedef typename ColorMap::property_value_type color_value;
  typedef graph_color<color_value>               color_set;

  vertex_iterator      vi_u, vi_v;
  typename Graph::adj_edge_iterator ei;
  vi_u =  g.find_vertex(s);
  assert(vi_u != g.end());
  color_map.put(*vi_u, color_set::gray());
  vis.discover_vertex(vi_u);
  ei = (*vi_u).begin();
  while ( ei != (*vi_u).end() ) {
    vertex_descriptor v = (*ei).target();
    vis.examine_edge(vi_u,ei);
    vi_v = g.find_vertex(v);
    if (vi_v != g.end()){
      color_value v_color = color_map.get(*vi_v);
      if ( v_color == color_set::white() ) {
        vis.tree_edge(vi_u,ei);
        depth_first_search(g, v, vis, color_map);
      }
      else if (v_color == color_set::gray()){
        vis.gray_target(vi_u,ei); /* back _edge*/
      }
      else if (v_color == color_set::black()){
        vis.black_target(vi_u,ei); /*cross edge*/
      }
      // else {std::cout<<"ERROR: colors in DFS functor"
      //                     <<std::endl; assert(false);}
    } // else std::cout << "\nIn GraphDFS: vid=" << v << " not in graph";
    ++ei;
  }
  color_map.put(*vi_u, color_set::black());
  vis.finish_vertex(vi_u);
}

///////////////////////////////////////////////////////////////////////////
/// @brief Performs a DFS on an input graph. No starting vertex
/// is specified. Instead, a DFS is run with different starting vertices
/// until all vertices have been reached by some DFS. Useful for performing
/// a DFS on a forest.
/// @param g The input graph.
/// @param vis The visitor class used to visit each vertex.
/// @param color_map The color map to be used, stores a color for each
/// vertex in the graph.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <class Graph, class DFSVisitor,class ColorMap>
void depth_first_search(Graph& g,
      DFSVisitor& vis,
      ColorMap& color_map)
{
  typedef typename Graph::vertex_iterator    vertex_iterator;
  typedef typename ColorMap::property_value_type  color_value;
  typedef graph_color<color_value>                color_set;
  //for all vertices check if it was touched; if not perform dfs
  for (vertex_iterator vi = g.begin(); vi != g.end(); ++vi) {
        if (color_map.get(*vi) == color_set::white())
          depth_first_search(g, (*vi).descriptor(), vis, color_map);
  }
}
///////////////////////////////////////////////// DFS_EQ


///////////////////////////////////////////////////////////////////////////
/// @brief Performs a DFS on an input graph. This
/// function allows for the visitor class to return an EARLY_QUIT value,
/// which will exit the DFS immediately. The function returns EARLY_QUIT if
/// the DFS is stopped early, and CONTINUE otherwise.
/// @param g The input graph.
/// @param s The starting vertex for the DFS.
/// @param vis The visitor class to be used.
/// @param color_map The color map to be used, which stores a color for each
/// vertex.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <class Graph, class DFSVisitor,class ColorMap>
visitor_return depth_first_search_early_quit(Graph& g,
                                   typename Graph::vertex_descriptor s,
                                   DFSVisitor& vis,
                                   ColorMap& color_map)
{
  typedef typename Graph::vertex_descriptor  vertex_descriptor;
  typedef typename Graph::vertex_iterator    vertex_iterator;
  typedef typename ColorMap::property_value_type  color_value;
  typedef graph_color<color_value>                color_set;

  vertex_iterator      vi_u, vi_v;
  typename Graph::adj_edge_iterator ei;
  vi_u =  g.find_vertex(s);
  assert(vi_u != g.end());
  color_map.put(*vi_u, color_set::gray());
  if (vis.discover_vertex(vi_u)==EARLY_QUIT)
    return EARLY_QUIT;
  ei = (*vi_u).begin();
  while ( ei != (*vi_u).end() ) {
    if (vis.examine_edge(vi_u,ei) ==EARLY_QUIT)
      return EARLY_QUIT;
    vertex_descriptor v = (*ei).target();
    vi_v = g.find_vertex(v);
    if (vi_v != g.end()){
      color_value v_color = color_map.get(*vi_v);
      if ( v_color == color_set::white() ) {
        if (vis.tree_edge(vi_u,ei) ==EARLY_QUIT)
          return EARLY_QUIT;
        if ( depth_first_search_early_quit(g, v, vis, color_map)
          == EARLY_QUIT )
          return EARLY_QUIT;
      }
      else if (v_color == color_set::gray()){
        if (vis.gray_target(vi_u,ei) == EARLY_QUIT)
          return EARLY_QUIT; /* back _edge*/
      }
      else if (v_color == color_set::black()){
        if (vis.black_target(vi_u,ei) == EARLY_QUIT)
          return EARLY_QUIT; /*cross edge*/
      }
      // else {cout<<"ERROR: colors in DFS functor"<<endl; assert(false);}
    } // else cout << "\nIn GraphDFS: vid=" << v << " not in graph";
    ++ei;
  }
  color_map.put(*vi_u, color_set::black());
  vis.finish_vertex(vi_u);
  return CONTINUE;
}

///////////////////////////////////////////////////////////////////////////
/// @brief Performs a DFS on an input graph. This
/// function allows for the visitor class to return an EARLY_QUIT value,
/// which will exit the DFS immediately. The function returns EARLY_QUIT if
/// the DFS is stopped early, and CONTINUE otherwise. No starting vertex is
/// specified. Instead, a DFS is performed with different starting vertices
/// until all vertices have been reached by some DFS.
/// @param g The input graph.
/// @param vis The visitor class used to visit each vertex.
/// @param color_map The color map used, which defines a color for each
/// vertex in the graph.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <class Graph, class DFSVisitor,class ColorMap>
visitor_return depth_first_search_early_quit(Graph& g,
      DFSVisitor& vis,
      ColorMap& color_map)
{
  typedef typename Graph::vertex_iterator        vertex_iterator;
  typedef typename ColorMap::property_value_type color_value;
  typedef graph_color<color_value>               color_set;
  //for all vertices check if it was touched; if not perform dfs
  for (vertex_iterator vi = g.begin(); vi != g.end(); ++vi) {
    if (color_map.get(*vi) == color_set::white())
      if (depth_first_search_early_quit(g, (*vi).descriptor(), vis, color_map)
        == EARLY_QUIT)
        return EARLY_QUIT;
  }
  return CONTINUE;
}

//***************************************************************
// depth first search using predecessors info; requires a directed
// with predecessors view
//***************************************************************

///////////////////////////////////////////////////////////////////////////
/// @brief Runs a DFS on an input graph using predecessors info.
/// @param g The input graph.
/// @param s The starting vertex for the DFS.
/// @param vis The predecessors DFS visitor class.
/// @param color_map The color map to be used for the DFS.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <class Graph, class PredsDFSVisitor,class ColorMap>
void depth_first_search_preds(Graph& g,
      typename Graph::vertex_descriptor s,
      PredsDFSVisitor& vis,
      ColorMap& color_map)
{

  typedef typename Graph::vertex_descriptor  vertex_descriptor;
  typedef typename Graph::vertex_iterator    vertex_iterator;

  typedef typename ColorMap::property_value_type  color_value;
  typedef graph_color<color_value>                color_set;

  vertex_iterator      vi_u, vi_v;
  typename Graph::preds_iterator ei;
  vi_u =  g.find_vertex(s);
  assert(vi_u != g.end());
  color_map.put(*vi_u, color_set::gray());
  vis.discover_vertex(vi_u);
  ei = (*vi_u).predecessors().begin();
  while ( ei != (*vi_u).predecessors().end() ) {
    vertex_descriptor v = *ei;
    vis.examine_edge(vi_u,*ei);
    vi_v = g.find_vertex(v);
    if (vi_v != g.end()){
      color_value v_color = color_map.get(*vi_v);
      if ( v_color == color_set::white() ) {       vis.tree_edge(vi_u,*ei);
        depth_first_search_preds(g, v, vis, color_map);
      }
      else if (v_color == color_set::gray()){
        vis.gray_target(vi_u,*ei); /* back _edge*/
      }
      else if (v_color == color_set::black()){
        vis.black_target(vi_u,*ei); /*cross edge*/
      }
      // else {cout<<"ERROR: colors in DFS functor"<<endl; assert(false);}
    } // else cout << "\nIn GraphDFS: vid=" << v << " not in graph";
    ++ei;
  }
  color_map.put(*vi_u, color_set::black());
  vis.finish_vertex(vi_u);
}


//**********************************************************
//                  Topological Traversal
//**********************************************************

///////////////////////////////////////////////////////////////////////////
/// @brief type-checker to see if an instantiation is a bidirectional view.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
///////////////////////////////////////////////////////////////////////////
template <class Graph>
struct is_bidirectional_view
{
  static const bool value=false;
};

//this can be specialized for succ/preds views

///////////////////////////////////////////////////////////////////////////
/// @brief Initializes the in-degree map for a topological
/// traversal.
/// @param g The input graph.
/// @param vis The visitor class to be used.
/// @param in_map The in-degree map to be used.
/// @param Q The buffer to be used to hold the vertices that have an in-degree
/// of zero.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
///////////////////////////////////////////////////////////////////////////
template <class Graph, class Visitor, class InDegreeMap, class Buffer, bool b>
struct _init_in_degree_map
{
  static void init(Graph& g, Visitor& vis, InDegreeMap& in_map, Buffer& Q){
    typename Graph::vertex_iterator vi, vi2;
    typename Graph::adj_edge_iterator   ei;
    for (vi = g.begin();vi != g.end();++vi){
      in_map.put(*vi,0);
    }
    for (vi = g.begin();vi != g.end();++vi){
      for (ei = (*vi).begin();ei != (*vi).end();++ei){
        vi2 = g.find_vertex((*ei).target());
        in_map.put(*vi2, in_map.get(*vi2) + 1) ;
      }
    }
    for (vi = g.begin();vi != g.end();++vi){
      if (in_map.get(*vi) == 0){
        vis.discover_vertex(vi);
        Q.push_back(vi);
      }
    }
  }
};

///////////////////////////////////////////////////////////////////////////
/// @brief Initializes an in-degree map for an input graph,
/// to be used in a topological sort. This function defines the behavior
/// when the boolean value in the template parameters is true. Instead of
/// manually finding the in-degree of each node, this function assumes that
/// the vertices have a list of their predecessors, and the in-degree is taken
/// as the size of this list.
/// @param g The input graph.
/// @param vis The visitor class to be used.
/// @param in_map The in-degree map to be initialized.
/// @param Q The buffer used to hold the vertices with an in-degree of zero.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
///////////////////////////////////////////////////////////////////////////
template <class Graph, class Visitor, class InDegreeMap, class Buffer>
struct _init_in_degree_map<Graph, Visitor, InDegreeMap, Buffer,true>
{
  static void init(Graph& g, Visitor& vis, InDegreeMap& in_map, Buffer& Q)
  {
    typename Graph::vertex_iterator vi;
    typename Graph::adj_edge_iterator   ei;
    for (vi = g.begin();vi != g.end();++vi){
      size_t in_d = (*ei).predecessors().size();
      in_map.put(*vi,in_d);
      if (in_d == 0){
        vis.discover_vertex(vi);
        Q.push_back(vi);
      }
    }
  }
};

///////////////////////////////////////////////////////////////////////////
/// @brief  Runs a topological traversal on an input graph. The
/// function first initializes an in-degree map, and then uses this to run a
/// topological traversal on the input graph.
/// @param g The input graph.
/// @param vis The visitor class to be used.
/// @param in_map The in-degree map to be used.
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////
template <class Graph, class Visitor,class InDegreeMap>
void  topological_traversal(Graph& g, Visitor& vis, InDegreeMap& in_map)
{
  typedef typename Graph::vertex_iterator    vertex_iterator;
  vertex_iterator vi, v;
  std::deque<vertex_iterator> Q;//queue used for trimer traversals

  //initializing the predecessors(in_degree) info is specialized
  //depending on the input view;
  _init_in_degree_map<Graph, Visitor, InDegreeMap,std::deque<vertex_iterator>,
                is_bidirectional_view<Graph>::value>::init(g, vis, in_map, Q);

  while (Q.size() > 0){//while there are more vertices
    //pop a vertex from Q
    v = Q.front();
    Q.pop_front();
    vis.examine_vertex(v);
    //for every child u of v
    for (typename Graph::adj_edge_iterator ei = (*v).begin();ei !=
                                               (*v).end(); ++ei){
      vi = g.find_vertex((*ei).target());
      if (vi != g.end()){
        vis.examine_edge(vi,ei);
        if ( in_map.get(*vi) == 0 ) {
          std::cout<<"Check this if in TOPO_traversal"<<std::endl;
          continue;
        }
        size_t in_d = in_map.get(*vi) - 1;
        in_map.put(*vi, in_d);
        if (in_d == 0){
          vis.tree_edge(vi,ei);
          vis.discover_vertex(vi);
          Q.push_back(vi);
        }
      }
    }//for all adjacent edges
  }//while
}//method TOPO_traversal

}//namespace sequential
}//namespace stapl

#endif
