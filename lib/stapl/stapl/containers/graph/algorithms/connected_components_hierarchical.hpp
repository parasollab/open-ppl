/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_P_CC_STATS_HV_H
#define STAPL_CONTAINERS_GRAPH_P_CC_STATS_HV_H

//////////////////////////////////////////////////////////////////////
/// @file
///
/// @brief Contains connected component algorithms implemented using
/// the statically typed hierarchical graph view.  There are no tests
/// of these algorithms at this time.
//////////////////////////////////////////////////////////////////////

#include <stapl/containers/graph/algorithms/connected_components.hpp>

#include <stapl/views/native_view.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include <stapl/containers/array/static_array.hpp>

#include <stapl/views/hierarchical_view_static.h> // MISSING
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/views/explicit_view.hpp>
#include <stapl/views/vector_view.hpp>

#include <graph/algorithms/graph_algo_util.h>
#include <graph/algorithms/breadth_first_search.h>
#include <stapl/containers/vector/vector.hpp>
#include <utility>

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include <stapl/containers/graph/algorithms/stripped_pbfs.hpp> // MISSING

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Work function that extracts all of the connected components
/// from a native partition of a graph, creates domains that contain
/// all of the vertices in the same CC, and adds these domains to
/// a distributed vector that will be used for the partition of
/// the next level of the hierarchical graph.
///
/// @tparam Map Property map for storing connected component IDs
/// @tparam Init Vector to add subdomains to
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Map, typename Init>
struct populate_distributed_domain
{
  Map* m_map;
  Init* m_init_vec;

  typedef void result_type;

  populate_distributed_domain(Map* map, Init* init)
    : m_map(map), m_init_vec(init)
  { }

  template<typename Graph>
  void operator()(Graph graph)
  {
    typedef domset1D<typename Graph::vertex_descriptor> domain_type;
    typedef boost::unordered_multimap<
      typename Map::value_type, typename Graph::vertex_descriptor> cc_map_type;

    cc_map_type cc_map;
    boost::unordered_set<typename Map::value_type> ccs;

    // save all ccs in a set
    for (typename Graph::vertex_iterator it = graph.begin();
         it != graph.end(); ++it)
      ccs.insert(m_map->get(*it));

    // populate multimap of vertex descriptors to their cc
    for (typename Graph::vertex_iterator it = graph.begin();
         it != graph.end(); ++it)
      cc_map.insert(std::make_pair(m_map->get(*it), (*it).descriptor()));


    // go through each cc id
    for (typename boost::unordered_set<
           typename Map::value_type>::iterator cc_it = ccs.begin();
         cc_it != ccs.end(); ++cc_it)
    {
      domain_type dom;

      // find all of the vertices that belong to this cc and add it to
      // the domain
      std::pair<typename cc_map_type::iterator,
                typename cc_map_type::iterator> range =
        cc_map.equal_range(*cc_it);

      for (typename cc_map_type::iterator it = range.first;
           it != range.second; ++it)
        dom += it->second;

      m_init_vec->add(dom);
    }
  }

  void define_type(typer& t)
  {
    t.member(m_map);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A visitor used by a sequential breadth-first search traversal
/// to populate a property map with a given ID.
///
/// @tparam Type of the sequential graph
/// @tparam Container Property map to populate
/// @see mark_vertices
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <class GRAPH,class CONTAINER>
class visitor_p_cc
 : public visitor_base<GRAPH>
{
  CONTAINER m_map;
  typename GRAPH::vertex_descriptor m_start_id;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the visitor with the property map and the ID to
  /// assign all reachable vertices.
  ///
  /// @param map A property map of IDs
  /// @param The ID to mark all reachable vertices.
  //////////////////////////////////////////////////////////////////////
  visitor_p_cc(GRAPH& , CONTAINER const& map,
               typename GRAPH::vertex_descriptor s)
    : m_map(map), m_start_id(s)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Visitation policy when discovering a vertex in breadth-first
  /// search ordering.
  ///
  /// @param vi Iterator to the vertex that is being visited
  //////////////////////////////////////////////////////////////////////
  inline  visitor_return discover_vertex(typename GRAPH::vertex_iterator vi)
  {
    m_map.put(*vi, m_start_id);
    return CONTINUE;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Mark all vertices reachable from a given source vertex with
/// a specific identifier in the given map.
///
/// @param g A sequential graph to traverse
/// @param map A map that will be filled based on reachability from the
/// source
/// @param cmap A color map used internally by the traversal
/// @param src A vertex from which to start the traversal
/// @param mark An ID with which to mark vertices.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Graph, typename Map, typename CMap>
void mark_vertices(Graph& g, Map& map, CMap& cmap,
                   typename Graph::vertex_descriptor src,
                   typename Graph::vertex_descriptor mark)
{
  visitor_p_cc<Graph,Map> vis(g, map, mark);
  breadth_first_search(g, src, vis, cmap, g.domain());
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function that computes sequential connected components
/// of a native partition of a graph.
///
/// @tparam Map Property map for storing connected component IDs
/// @bug This algorithm will not work without hacking the sequential BFS
/// to do a domain::contains() edge to skip remote edges. Ideally, we would
/// want to somehow create a view that doesn't contain remote edges.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename Map>
class seq_cc_stats_wf
{
  Map m_map;

public:
  typedef void result_type;

  seq_cc_stats_wf(Map const& map)
    : m_map(map)
  { }

  void define_type(typer& t)
  {
    t.member(m_map);
  }

  template<typename Graph>
  void operator()(Graph g)
  {
    typedef typename Graph::vertex_iterator VI;

    // initialize the map:
    for (VI vi = g.begin(); vi != g.end(); ++vi) {
      m_map.put(*vi, INT_MAX);
    }

    // mark each CC with the id of the leading vertex:
    sequential::map_property_map<Graph, size_t> cmap;
    for (VI vi = g.begin(); vi != g.end(); ++vi) {
      if (m_map.get(*vi) == INT_MAX){
        mark_vertices(g, m_map, cmap, (*vi).descriptor(), (*vi).descriptor());
        cmap.reset();
        m_map.put(*vi, (*vi).descriptor());
      }
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Partition functor to create supervertices in a hierarchical
/// graph such that if two vertices were in the same connected component
/// at a lower level, they should be in the same supervertex at the
/// next level.
///
/// This is accomplished by running a sequential connected components
/// algorithm on native partitions of the graph.
///
/// @tparam G Type of the hierarchical graph
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<class G>
struct HPartitioner
{
  typedef part_explicit<
             typename G::domain_type,
             typename G::domain_type,
             stapl::vector<domset1D<typename G::vertex_descriptor> >
          >                                                   explicit_part;

  typedef typename stapl::result_of::explicit_view::type      view_type;

  template<class GV, class PMap>
  view_type operator()(GV& g, PMap map, int level)
  {
    typedef domset1D<typename GV::vertex_descriptor> setdom_type;

    ::stapl::map_func(seq_cc_stats_wf<PMap>(map), stapl::native_view(g));

    typedef stapl::vector<setdom_type> init_vec_type;
    init_vec_type      init_vec;
    vector_view<init_vec_type> init_vec_view(init_vec);

    // create explicit domains independently on each location -- must
    // fence after this to synchronize!
    ::stapl::map_func(populate_distributed_domain<PMap, init_vec_type>(
                                      &map, &init_vec), stapl::native_view(g));
    init_vec.distribution().synchronize_metadata();

    // create partitions; must fence after this to synchronize!
    explicit_part part_exp(g.domain(), init_vec);

    return stapl::explicit_view(g, part_exp);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function that updates the previous level's property map
/// to set their CCs to the current level's VDs. This is the fine-grained
/// work function that operates on vertices.
///
/// @see update_supervertex_map
/// @tparam Map Property map for connected components
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Map>
struct set_cc
{
  Map m_map;
  size_t m_cc;

  typedef void result_type;

  set_cc(Map map, size_t cc )
    : m_map(map), m_cc(cc)
  { }

  template<typename Vertex>
  void operator()(Vertex v)
  {
    m_map.put(v.descriptor(), m_cc);
  }

  void define_type(typer& t) { t.member(m_map); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function that updates the previous level's property map
/// to set their CCs to the current level's VDs. This is the coarse-grained
/// work function that operates on supervertices.
///
/// @tparam Map Property map for connected components
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Map>
struct update_supervertex_map
{
  Map m_map;

  typedef void result_type;

  update_supervertex_map(Map map)
    : m_map(map)
  { }

  template<typename SuperVertex>
  void operator()(SuperVertex v)
  {
    stapl::map_func(set_cc<Map>(m_map, v.descriptor()), v.property());
  }

  void define_type(typer& t) { t.member(m_map); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function that adds edges to the top level graph based
/// on edge information in the lower level graph.
///
/// @tparam Graph Outer graph (top level graph)
/// @tparam Map Property map for connected components
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Graph, typename Map>
struct add_edge_func
{
  Map m_map;
  Graph m_graph;
  size_t m_source;

  add_edge_func() { }

  add_edge_func(const Map& map, const Graph& graph, size_t source)
    : m_map(map), m_source(source)
  { }

  template<typename Vertex>
  void operator()(Vertex v) const
  {
    typename Vertex::vertex_descriptor target_cc = m_map.get(v.descriptor());
    if (target_cc != m_source) {
      typename Graph::view_container_type* cont =
        const_cast<typename Graph::view_container_type*>(
                                              m_graph.get_container());
      cont->add_edge_async(typename Graph::edge_descriptor(
                                              target_cc, m_source));
    }
  }

  void define_type(typer& t)
  {
    t.member(m_map);
    t.member(m_graph);
    t.member(m_source);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function that pushes requests along edges of the lower
/// graph to add edges to the supervertices that the edges' targets
/// are in.
/// @ingroup pgraphAlgoDetails
///
/// @tparam Map Property map for connected components
/// @todo Why is the add_edge commented out?
//////////////////////////////////////////////////////////////////////
template<typename Map>
struct add_edge_to_cc
{
  Map m_map;
  size_t source;

  typedef void result_type;

  add_edge_to_cc(Map map, size_t src)
    : m_map(map), source(src)
  { }

  template<typename Vertex, typename Graph, typename OuterGraph>
  void operator()(Vertex v, Graph g, OuterGraph og)
  {
    for (typename Vertex::adj_edge_iterator it = v.begin(); it != v.end(); ++it)
    {
      g.apply_set((*it).target(),
                  add_edge_func<OuterGraph, Map>(m_map, og, source));
    }

//    og.add_edge_async(source, (source + 1) % g.size());
  }

  void define_type(typer& t) { t.member(m_map); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function that adds edges between supervertices that
/// represent connected components.
/// @ingroup pgraphAlgoDetails
///
/// @tparam Map Property map for connected components
/// @todo Why is the add_edge commented out?
//////////////////////////////////////////////////////////////////////
template<typename Map>
struct propagate_edges
{
  Map m_map;

  typedef void result_type;

  propagate_edges(Map map)
    : m_map(map)
  { }

  template<typename SuperVertex, typename Graph>
  void operator()(SuperVertex v, Graph g)
  {
    stapl::map_func(add_edge_to_cc<Map>(m_map, v.descriptor()), v.property(),
                    make_repeat_view(v.property()), make_repeat_view(g));
    //    g.add_edge_async(typename Graph::edge_descriptor(v.descriptor(),
    //                                       (v.descriptor() + 1) % g.size()));
  }

  void define_type(typer& t) { t.member(m_map); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Edge functor that is used to connect edges between supervertices
/// in a hierarchical graph such that if there is an edge between any
/// two vertices in the previous level, there will be an edge between
/// their two supervertices.
/// @ingroup pgraphAlgoDetails
///
/// @tparam Map Property map for connected components
//////////////////////////////////////////////////////////////////////
template<typename Map>
struct EF
{
  Map m_map;

  EF(Map map)
    : m_map(map)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add edges between supervertices at a given level.
  ///
  /// @param g Pointer to a graph
  /// @param lvl Level of the hierarchical graph to which to add edges.
  //////////////////////////////////////////////////////////////////////
  template<class Graph>
  void operator()(Graph* g, size_t lvl) const
  {
    graph_view<Graph> view(*g);

    // update the previous level's property map to set their CCs
    // to the current level's VDs
    stapl::map_func(update_supervertex_map<Map>(m_map), view);

    // go through each vertex in all supervertices and add edges from the
    // vertex to the target's CC
    stapl::map_func(propagate_edges<Map>(m_map), view, make_repeat_view(view));
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Given a graph, mark all vertices belonging to the same
/// connected component in a property map.
/// @ingroup pgraphAlgo
///
/// @param g A view of the graph.
/// @todo This should take a property map as parameter and populate it,
/// instead of assuming the vertex has the necessary property interface.
//////////////////////////////////////////////////////////////////////
template<typename View>
void p_cc_stats(View& g)
{
  typedef domset1D<typename View::vertex_descriptor> gdom_t;
  typedef graph_view<typename View::view_container_type, gdom_t>   graph_view_t;
  graph_view_t vgraph(g.container(), gdom_t(0, g.num_vertices()-1));

  typedef static_array<size_t>                            property_storage_type;

  typedef graph_external_property_map<
    View, size_t, property_storage_type>                  property_map_type;

  typedef domset1D<typename View::vertex_descriptor>      setdom_type;

  property_map_type map(g, new property_storage_type(g.size()));

  typedef create_hierarchy<graph_view_t, HPartitioner, 1> CHT;
  typename CHT::value_type hv = CHT()(vgraph, EF<property_map_type>(map), map);
  connected_components_fine(hv);
}


//////////////////////////////////////////////////////////////////////
/// @brief Given a graph and a connected component map of vertices,
/// determine if two vertices are in the same connected component.
///
/// This has the assumption that a map is first populated using the
/// @ref p_cc_stats() algorithm.
///
/// @param g A View of the graph
/// @param map A property map of connected component IDs
/// @param v1, v2 Two vertices to query.
/// @return True if the two input vertices are in the same connected
/// component.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GView, typename CCMap, typename VertexDescriptor>
bool p_is_same_cc(GView& g, CCMap& map, VertexDescriptor v1,
                  VertexDescriptor v2)
{
  return (map.get(*g.find_vertex(v1)) == map.get(*g.find_vertex(v2)));
}

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_P_CC_STATS_HV
