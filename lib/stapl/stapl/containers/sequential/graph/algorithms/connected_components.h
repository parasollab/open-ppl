/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_CONNECTED_COMPONENTS_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_CONNECTED_COMPONENTS_HPP

#include <vector>
#include <limits>
#include <algorithm>
#include "breadth_first_search.h"

namespace stapl{
namespace sequential{

//////////////////////////////////////////////////////////////////////////
/// @brief Determine which connected component the given vertex is in.
/// @tparam Graph the graph
/// @return EARLY_QUIT if the connected component is found. Otherwise CONTINUE
/// @ingroup seqGraphWf
/// @todo move into a detail namespace
//////////////////////////////////////////////////////////////////////////
template <class Graph>
class visitor_samecc
        : public visitor_base<Graph>
{
    typedef typename Graph::vertex_descriptor VD;
    /// the vertex needing identification
    VD v2id;
public:
  visitor_samecc(Graph& , VD _v2)
    : v2id(_v2)
  { }
  visitor_return discover_vertex(typename Graph::vertex_iterator v)
  {
    if ((*v).descriptor() == v2id) return EARLY_QUIT;
    else return CONTINUE;
  }
};

///////////////////////////////////////////////////////////////////////////////
/// @brief Check if vid1 and vid2 are in the same component.
/// @param _g the graph
/// @param _color_map the map of each vertex's color
/// @param _v1id the source vertex
/// @param _v2id the destination vertex
/// @return true if vid1 and vid2 are in the same component, false if otherwise
/// @see BFS(VID)
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////////////
template <class Graph, class ColorMap>
bool is_same_cc (Graph& _g, ColorMap& _color_map,
                 typename Graph::vertex_descriptor _v1id,
                 typename Graph::vertex_descriptor _v2id)
{
  visitor_samecc<Graph> vis(_g, _v2id);
  if (breadth_first_search_early_quit(_g, _v1id, vis, _color_map) == EARLY_QUIT)
    return true;
  else return false;
}

//////////////////////////////////////////////////////////////////
/// @brief Add the current vertex to the connected component's
/// vector of vertex descriptors
/// @tparam Graph the graph type
/// @tparam CONTAINER container type for storing the vertex descriptors
/// @return CONTINUE
/// @ingroup seqGraphAlgoWf
/// @todo move into a detail namespace
//////////////////////////////////////////////////////////////////
template <class Graph,class CONTAINER>
class visitor_cc
        : public visitor_base<Graph>
{
 public:
  // container that holds the vertex descriptors
  CONTAINER& ccverts;
  visitor_cc(Graph& , CONTAINER& _v)
    : ccverts(_v)
  { }
  visitor_return discover_vertex(typename Graph::vertex_iterator vi)
  {
    ccverts.push_back((*vi).descriptor());
    return CONTINUE;
  }
};

//////////////////////////////////////////////////////////////
/// @brief A comparator work function that returns x>y.
/// @tparam T the operands' type
/// @param x, y the objects to be compared
/// @ingroup seqGraphAlgoWf
/// @todo move into a detail namespace
//////////////////////////////////////////////////////////////
template <class T>
struct __CCVID_Compare
         : public std::binary_function<T, T, bool>
{
  bool operator()(T x, T y)
  {
    return x.first > y.first;
  }
};

////////////////////////////////////////////////////////
/// @brief Get a list of vertices which are in the same connected
/// component with the specified vertex.
/// @param _g the graph
/// @param _color_map the map of each vertex's color
/// @param _v1id the vertex id that defines the connected component
/// @param _ccverts container(std::vector) where the vertices' ids will be
/// stored
/// @param sort_results Indicates whether the vertices' ids will be sorted
/// @return the number of elements pushed in _ccverts
/// @see BFS(VERTEX&)
/// @ingroup seqGraphAlgo
//////////////////////////////////////////////////////////
template<class Graph, class ColorMap>
size_t get_cc ( Graph& _g, ColorMap& _color_map,
                typename Graph::vertex_descriptor _v1id,
                std::vector<typename Graph::vertex_descriptor>& _ccverts,
                bool sort_results = false)
{
  _ccverts.clear();
  visitor_cc<Graph,std::vector<typename Graph::vertex_descriptor> >
    vis(_g, _ccverts);
  breadth_first_search(_g, _v1id, vis, _color_map);
  if (sort_results) std::stable_sort(_ccverts.begin(),_ccverts.end());
  return _ccverts.size();
}

/////////////////////////////////////////////////////////////
/// @brief Get two lists: one that matches each vertex with the size
/// of its connected component and one that lists the members of each
/// connected component.
/// @param _g the graph
/// @param _color_map the map of each vertex's color
/// @param _ccstats the container(std::vector) to store the pairs(size of
/// connected component, vertex)
/// @param _ccverts the container(std::vector) of vectors to store the list of
/// vertices
/// @return the number connected components pushed into _ccstats
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////
template <class Graph,class ColorMap>
size_t get_cc_stats (Graph& _g,
                     ColorMap& _color_map,
                     std::vector<
                       std::pair<size_t,typename Graph::vertex_descriptor>
                     >& _ccstats,
                     std::vector<
                       std::vector<typename Graph::vertex_descriptor>
                     >& _ccverts)
{
  typedef typename ColorMap::property_value_type  color_value;
  typedef graph_color<color_value>                color_set;

  _ccstats.clear();
  _ccverts.clear();

  for (typename Graph::vertex_iterator vi = _g.begin(); vi != _g.end(); ++vi) {
    if (_color_map.get(*vi) == color_set::white()){
      std::vector<typename Graph::vertex_descriptor> _ccverts_local;
      visitor_cc<Graph,std::vector<typename Graph::vertex_descriptor> >
        vis(_g, _ccverts_local);
      breadth_first_search(_g, (*vi).descriptor(), vis, _color_map);
      _ccstats.push_back(std::pair<size_t,typename Graph::vertex_descriptor>
                           (vis.ccverts.size(),(*vi).descriptor()));
      _ccverts.push_back(vis.ccverts);
    }
  }

  return _ccstats.size();
}

///////////////////////////////////////////////////////////
/// @brief Get two lists: one that contains a pair of the identifier vertex
/// with the size of its connected component and one that lists the members of
/// each connected component.
/// @param _g the graph
/// @param _color_map the map of each vertex's color
/// @param _ccstats the container(std::vector) to store the pairs(size of
/// connected component, vertex)
/// @param _ccverts the container(std::vector) of vectors to store the list of
/// vertices
/// @param first_k indicate the maximum number of connected components
/// @param sort_results indicate whether or not to sort _ccstats
/// @return the number connected components pushed in _ccstats
/// @ingroup seqGraphAlgo
////////////////////////////////////////////////////////////
template <class Graph,class ColorMap>
size_t get_cc_stats (Graph& _g,
                     ColorMap& _color_map,
                     std::vector<
                       std::pair<size_t,typename Graph::vertex_descriptor>
                     >& _ccstats,
                     std::vector<
                       std::vector<typename Graph::vertex_descriptor>
                     >& _ccverts,
                     size_t first_k,
                     bool sort_results = false)
{
  get_cc_stats(_g, _color_map, _ccstats, _ccverts);

  typedef typename Graph::vertex_descriptor VID;
  if (sort_results)
    std::sort(_ccstats.begin(),
              _ccstats.end(),
              __CCVID_Compare<std::pair<int,VID> >() );
  if (first_k < _ccstats.size())
    _ccstats.resize(first_k);
  return _ccstats.size();
}

///////////////////////////////////////////////////
/// @brief Count the number of vertices in the connected component.
/// @tparam Graph the graph
/// @return the number of vertices in the connected component
/// @ingroup seqGraphAlgoWf
/// @todo move into a detail namespace
//////////////////////////////////////////////////
template <class Graph>
class visitor_cc_count
        : public visitor_base<Graph>
{
 public:
  /// the number of vertices in the connected component
  size_t& ccverts;

  visitor_cc_count(size_t& _cc) : ccverts(_cc) { }
  visitor_return discover_vertex(typename Graph::vertex_iterator) {
    ccverts++;
    return CONTINUE;
  }
};

///////////////////////////////////////////////////////////
/// @brief Get all connected components from the graph.
/// @param _g the graph
/// @param _color_map the map of each vertex's color
/// @param _ccstats the container(std::vector) to store the pairs(size of
/// connected component, vertex)
/// @return the number of components pushed in _ccstats
/// @note The list is ordered by the size of component, the first element in
/// the list has largest size.
/// @see CCVID_Compare
/// @ingroup seqGraphAlgo
////////////////////////////////////////////////////////////
template <class Graph,class ColorMap>
size_t get_cc_stats (Graph& _g,
                     ColorMap& _color_map,
                     std::vector<
                       std::pair<size_t,typename Graph::vertex_descriptor>
                     >& _ccstats)
{
  typedef typename ColorMap::property_value_type  color_value;
  typedef graph_color<color_value>                color_set;

  _ccstats.clear();
  size_t ccs;
  visitor_cc_count<Graph> vis(ccs);
  for (typename Graph::vertex_iterator vi = _g.begin(); vi != _g.end(); ++vi) {
    if (_color_map.get(*vi) == color_set::white()){
      ccs=0;
      breadth_first_search(_g, (*vi).descriptor(), vis, _color_map);
      _ccstats.push_back(std::pair<size_t,typename Graph::vertex_descriptor>
                           (vis.ccverts,(*vi).descriptor()));
    }
  }

  return _ccstats.size();
}

/////////////////////////////////////////////////////////
/// @brief Get all connected components from the graph.
/// @param _g the graph
/// @param _color_map the map of each vertex's color
/// @param _ccstats std::vector where the statistic will be stored
/// @param first_k the maximum number of connected components allowed
/// @param sort_results indicate whether or not to sort _ccstats
/// @return the number of components pushed in _ccstats
/// @note The list is ordered by the size of component, the first element in
/// the list has largest size.
/// @see CCVID_Compare
/// @ingroup seqGraphAlgo
////////////////////////////////////////////////////////////
template <class Graph,class ColorMap>
size_t get_cc_stats (Graph& _g,
                     ColorMap& _color_map,
                     std::vector<
                       std::pair<size_t,typename Graph::vertex_descriptor>
                     >& _ccstats,
                     size_t first_k,
                     bool sort_results = false)
{
  typedef typename Graph::vertex_descriptor VID;
  get_cc_stats(_g,_color_map,_ccstats);
  if (sort_results)
    std::sort(_ccstats.begin(),
              _ccstats.end(),
              __CCVID_Compare<std::pair<int,VID> >() );
  if (first_k < _ccstats.size())
    _ccstats.resize(first_k);
  return _ccstats.size();
}

////////////////////////////////////////////////////////////
/// @brief Get the number of connected components in the graph.
/// @param _g the graph
/// @param _color_map the map of each vertex's color
/// @return the number of connected components
/// @see get_cc_stats
/// @ingroup seqGraphAlgo
////////////////////////////////////////////////////////////
template <class Graph, class ColorMap>
size_t get_cc_count (Graph& _g, ColorMap& _color_map)
{
  std::vector< std::pair<size_t,typename Graph::vertex_descriptor> > ccstats;
  return get_cc_stats(_g, _color_map, ccstats);
}

///////////////////////////////////////////////////////////////
/// @brief Store and provide access to the edges between the connected
/// components.
/// @tparam Graph the graph
/// @tparam CONTAINER stores the edges
/// @ingroup seqGraphUtil
////////////////////////////////////////////////////////////////
template <class Graph,class CONTAINER>
class _visitor_cc_edges
        : public visitor_base<Graph>
{
  typedef typename CONTAINER::value_type VT;
  /// the container that holds the graph's edges
  CONTAINER& ccedges;

 public:
  _visitor_cc_edges(Graph& , CONTAINER& _v)
    : ccedges(_v)
    { }

  visitor_return tree_edge(typename Graph::vertex_iterator,
                                  typename Graph::adj_edge_iterator ei)
  {
    VT nextedge((*ei).source(), (*ei).target());
    ccedges.push_back(nextedge);
    return CONTINUE;
  }

  visitor_return gray_target(typename Graph::vertex_iterator,
                                    typename Graph::adj_edge_iterator ei)
  {
    VT nextedge((*ei).source(), (*ei).target());
    ccedges.push_back(nextedge);
    return CONTINUE;
  }
  visitor_return black_target(typename Graph::vertex_iterator,
                                     typename Graph::adj_edge_iterator ei)
  {
    VT nextedge((*ei).source(), (*ei).target());
    ccedges.push_back(nextedge);
    return CONTINUE;
  }
};

////////////////////////////////////////////////////////////////////////
/// @brief Compare two values
/// @tparam PVID the operand type
/// @return true if the first operand is smaller than the second and false if
/// otherwise
/// @ingroup seqGraphAlgoWf
/// @todo move into a detail namespace
/////////////////////////////////////////////////////////////////////////
template<class PVID>
struct _compare_pair_vid
{
  bool operator () (PVID a, PVID b)
  {
    return a.first < b.first;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Get the edges in the connected component defined by the specified
/// vertex(unweighted).
/// @param _g the graph
/// @param _color_map map of each vertex's color
/// @param _ccedges container(vector) where the edges will be stored
/// @param _vd the vertex id that defines the connected component
/// @note The returned edge list may contain duplicate edges. This occurs
/// when vid1->vid2 was in the list and vid2->vid1 might also be in the list.
/// @return the number of elements pushed in _ccedges
/// @see BFS(VID) and GetIncidentEdges
/// @ingroup seqGraphAlgo
///////////////////////////////////////////////////////////////////////
template<class Graph,class ColorMap>
int get_cc_edges (Graph& _g,
                  ColorMap& _color_map,
                  std::vector<
                    std::pair<typename Graph::vertex_descriptor,
                              typename Graph::vertex_descriptor>
                  >& _ccedges,
                  typename Graph::vertex_descriptor _vd)
{
  typedef std::pair<typename Graph::vertex_descriptor,
                    typename Graph::vertex_descriptor> PVD;
  _ccedges.clear();
  _visitor_cc_edges<Graph,std::vector<PVD> > vis(_g,_ccedges);
  breadth_first_search(_g, _vd, vis, _color_map);
  _compare_pair_vid<PVD> _compare;
  std::stable_sort(_ccedges.begin(),_ccedges.end(),_compare);
  return _ccedges.size();
}


}//namespace sequential
}//namespace stapl

#endif
