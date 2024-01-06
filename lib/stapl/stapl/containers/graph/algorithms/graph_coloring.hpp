/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_GRAPH_COLORING_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_GRAPH_COLORING_HPP

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/domains/interval.hpp>
#include <stapl/containers/partitions/explicit.hpp>
#include <boost/unordered_map.hpp>
#include <vector>
#include <utility>

namespace stapl {

namespace graph_coloring_impl {


//////////////////////////////////////////////////////////////////////
/// @brief Functor to extract color of a remote-neighbor vertex.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct get_remote_neighbor_colors
{
  typedef boost::unordered_map<size_t, size_t> result_type;

private:
  size_t m_key;

public:
  get_remote_neighbor_colors(size_t const& key)
    : m_key(key)
  { }

  template <typename HashMap>
  result_type operator()(HashMap& hmap) const
  {
    typename HashMap::iterator it;
    it = hmap.find(m_key);

    if (it == hmap.end())
      return result_type();

    return it->second;
  }

  void define_type(typer& t)
  {
    t.member(m_key);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to store received color from a neighbor vertex.
/// Used by @ref set_remote_neighbor_colors_of_vertex.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct set_remote_neighbor_colors
{
  typedef void result_type;

private:
  typedef std::pair<size_t, size_t> neighbor_color_type;

  size_t              m_vid;
  neighbor_color_type m_neighbor_color;

public:
  set_remote_neighbor_colors(size_t const& vid,
                             neighbor_color_type const& neighbor_color)
    : m_vid(vid), m_neighbor_color(neighbor_color)
  { }

  template <typename HashMap>
  void operator()(HashMap& hmap) const
  {
    hmap[m_vid][m_neighbor_color.first] = m_neighbor_color.second;
  }

  void define_type(typer& t)
  {
    t.member(m_vid);
    t.member(m_neighbor_color);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to inform the neighbors of a vertex of its color
/// using @ref set_remote_neighbor_colors.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename HashMap>
class set_remote_neighbor_colors_of_vertex
{
public:
  typedef std::pair<size_t, size_t> neighbor_color_type;
  typedef void                      result_type;

private:
  neighbor_color_type               m_neighbor_color;
  p_object_pointer_wrapper<HashMap> m_hashmap;

public:
  set_remote_neighbor_colors_of_vertex(
    neighbor_color_type const& neighbor_color,
    HashMap& hashmap)
    : m_neighbor_color(neighbor_color),
      m_hashmap(&hashmap)
  { }

  template <typename Vertex>
  void operator()(Vertex&& v) const
  {
    m_hashmap->apply_set(
      m_hashmap->get_location_id(),
      set_remote_neighbor_colors(v.descriptor(), m_neighbor_color)
    );
  }

  void define_type(typer& t)
  {
    t.member(m_neighbor_color);
    t.member(m_hashmap);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function to pick a color that does not conflict with the
/// provided colors. Used to pick a safe color for a vertex.
/// @param neighbor_colors A domain specifying the colors of all neighbors
/// of the vertex in question.
/// @return A safe color that can be assigned to the vertex without creating
/// conflicts with the input set of colors.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
inline size_t pick_vertex_color(domset1D<size_t> const& neighbor_colors)
{
  size_t num_colors = neighbor_colors.size();
  domset1D<size_t> contiguous_dom(num_colors);
  contiguous_dom -= neighbor_colors;
  if (contiguous_dom.empty())
    return num_colors;
  else
    return contiguous_dom.first();
}


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to compute graph coloring.
/// Works over a partition of the graph to compute the coloring for
/// each partition independently. The coloring for border vertices is
/// is then adjusted to reflect any colors received from neighbors in
/// a different partition.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct graph_coloring_wf
{
  typedef void result_type;

  template <typename Partition, typename GView, typename ColorMap,
            typename Array>
  void operator()(Partition const& p, GView& gview, ColorMap& color_map,
                  Array& hashmap_array) const
  {
    typedef boost::unordered_map<size_t, size_t> color_boundary_map_type;
    typename Partition::const_vertex_iterator it = p.begin(),
                                          end_it = p.end();
    for (; it!=end_it; ++it) {
      bool boundary_vertex = false;
      domset1D<size_t> neighbor_colors;
      std::vector<size_t> remote_neighbors;
      size_t vid = (*it).descriptor();
      typename Partition::const_vertex_iterator::reference::adj_edge_iterator
        edge_it = (*it).begin(), edge_end_it = (*it).end();
      for (; edge_it!=edge_end_it; ++edge_it) {
        size_t target = (*edge_it).target();

        //Get colors of local neighbors
        if (p.domain().contains(target) && (target != vid)) {
          neighbor_colors += color_map.get(target);
        } else {
          remote_neighbors.push_back(target);
          boundary_vertex = true;
        }
      } //endfor over edges

      //Check received colors of remote neighbors
      if (boundary_vertex) {
        color_boundary_map_type bmap =
          hashmap_array.apply_get(
            hashmap_array.container().get_location_id(),
            get_remote_neighbor_colors(vid)
          );
        color_boundary_map_type::iterator map_it = bmap.begin(),
                                          map_end_it = bmap.end();
        for (; map_it!=map_end_it; ++map_it)
          neighbor_colors += map_it->second;
      }

      size_t mycolor = pick_vertex_color(neighbor_colors);
      //set my color
      color_map.put(vid, mycolor);

      //send my color to neighbors
      if (boundary_vertex) {
        std::vector<size_t>::iterator vect_it = remote_neighbors.begin(),
                                      vect_end_it = remote_neighbors.end();
        for (; vect_it!=vect_end_it; ++vect_it) {
          typedef typename view_traits<Array>::container hashmap_type;

          gview.apply_set(*vect_it,
                          set_remote_neighbor_colors_of_vertex<hashmap_type>(
                            std::make_pair(vid, mycolor),
                            hashmap_array.container()));
        }
      }
    } //endfor over the vertices
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to return if the coloring stored in the provided
/// vertex property map (color_map) is valid for the input graph.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct is_valid_graph_coloring_wf
{
  typedef bool result_type;

  template <typename Vertex, typename ColorMap>
  bool operator()(Vertex const& v, ColorMap& color_map) const
  {
    size_t vid = v.descriptor();
    size_t mycolor = color_map.get(v);
    typename Vertex::const_adj_edge_iterator edge_it = v.begin(),
                                             edge_end_it = v.end();
    for (; edge_it!=edge_end_it; ++edge_it)
    {
      size_t target = (*edge_it).target();
      if ((color_map.get(target) == mycolor) && (target != vid))
        return false;
    }
    return true;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to return the color of a vertex.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct get_vertex_color
{
  typedef domset1D<size_t> result_type;

  template <typename Vertex, typename ColorMap>
  result_type operator()(Vertex const& v, ColorMap& color_map) const
  {
    size_t mycolor = color_map.get(v);
    return result_type(mycolor, mycolor);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to merge two vectors. Used in reduce.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename T>
struct reduce_vector_wf
{
  typedef std::vector<T> result_type;
  template<typename Element1, typename Element2>
  result_type operator()(Element1 const& elt1, Element2 const& elt2)
  {
    result_type result = elt1;
    std::copy(elt2.begin(), elt2.end(), std::back_inserter(result));
    return result;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to return the sub-domain of the elements in
/// the hash-map.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename HashMap, typename DomainType>
struct get_sub_domain
{
  typedef std::vector<DomainType> result_type;
  template<typename Element>
  result_type operator()(Element const& elt)
  {
    HashMap hmap = elt;
    DomainType dom;
    typename HashMap::iterator it = hmap.begin(),
                           end_it = hmap.end();
    for (; it!=end_it; ++it)
    {
      dom += it->first;
    }
    return result_type(1, dom);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to resolve color conflicts between boundary
/// vertices that end up with the same colors despite being neighbors.
/// In such cases, one of the vertices is re-assigned its colors based
/// on the next safe color it can use.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct resolve_color_conflicts
{
  typedef bool result_type;

  template <typename Partition, typename GView, typename ColorMap,
            typename Array>
  result_type operator()(Partition const& p, GView& gview, ColorMap& color_map,
                         Array& hashmap_array) const
  {
    typedef boost::unordered_map<size_t, size_t> color_boundary_map_type;
    bool partition_conflict = false;
    typedef typename Partition::const_vertex_iterator const_vertex_iterator;
    const_vertex_iterator it = p.begin(), end_it = p.end();
    for (; it!=end_it; ++it) {
      bool responsible_for_conflict = false;
      domset1D<size_t> neighbor_colors;
      size_t vid = (*it).descriptor();
      size_t mycolor = color_map.get(vid);
      color_boundary_map_type bmap =
        hashmap_array.apply_get(
          hashmap_array.container().get_location_id(),
          get_remote_neighbor_colors(vid)
        );

      color_boundary_map_type::iterator map_it = bmap.begin(),
                                        map_end_it = bmap.end();
      //Compare my color with remote neighbors' colors
      for (; map_it!=map_end_it; ++map_it) {
        size_t neighbor_id = map_it->first;
        size_t neighbor_color = map_it->second;
        neighbor_colors += neighbor_color;
        if (mycolor == neighbor_color && (vid < neighbor_id)) {
          partition_conflict = true;
          responsible_for_conflict = true;
        }
      }

      if (responsible_for_conflict) {
        const auto p = *it;

        typename decltype(p)::const_adj_edge_iterator
          edge_it = p.begin(), edge_end_it = p.end();
        for (; edge_it!=edge_end_it; ++edge_it) {
          size_t target = (*edge_it).target();

          //Get colors of local neighbors
          if (!bmap.count(target) && (target != vid)) {
            neighbor_colors += color_map.get(target);
          }
        }

        size_t new_color = pick_vertex_color(neighbor_colors);

        //set my color
        color_map.put(vid, new_color);

        //send my new color to remote neighbors
        map_it = bmap.begin();
        for (; map_it!=map_end_it; ++map_it) {
          typedef typename view_traits<Array>::container hashmap_type;

          gview.apply_set(map_it->first,
                          set_remote_neighbor_colors_of_vertex<hashmap_type>(
                            std::make_pair(vid, new_color),
                            hashmap_array.container()));
        }
      }
    }
    return partition_conflict;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to populate an output undirected graph with
/// vertices from the directed graph.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct build_vertex_undirected_graph_wf
{
  typedef void result_type;

  template <typename Vertex,typename UGraph>
  void operator()(Vertex v, UGraph& ugraph) const
  {
    ugraph.add_vertex(v.descriptor(), typename UGraph::vertex_property());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to create an undirected version of a directed graph.
/// Adds bi-directional edges for each directed edge in the input graph.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct build_edge_undirected_graph_wf
{
  typedef void result_type;

  template <typename Vertex,typename UGraph>
  void operator()(Vertex v, UGraph& ugraph) const
  {
    typedef typename UGraph::edge_descriptor edge_descriptor;
    typename Vertex::adj_edge_iterator edge_it = v.begin(),
                                       edge_end_it = v.end();
    for (; edge_it!=edge_end_it; ++edge_it)
    {
      ugraph.add_edge_async(edge_descriptor((*edge_it).source(),
                                            (*edge_it).target()));
    }
  }
};

} // namespace graph_coloring_impl


//////////////////////////////////////////////////////////////////////
/// @brief Parallel Level-Synchronized Graph Coloring Algorithm.
///
/// Assigns coloring to the vertices of the input graph, such that
/// neighboring vertices are not assigned the same color.
/// @param graph The @ref graph_view over the input graph.
/// @param color_map The output vertex property map where the color information
/// will be stored. [vertex->color (int)]
/// Specialization for directed graphs.
/// Copies the directed graph to an undirected graph and computes coloring
/// for the undirected graph.
/// @warning  This method invalidates the input graph view.
/// @ingroup pgraphAlgo
/// @todo This could be using an @ref undirected_view over the input graph
/// instead of creating its own methods to make the undirected graph.
//////////////////////////////////////////////////////////////////////
template <template<typename, typename, typename, typename> class GView,
          template<graph_attributes, graph_attributes, typename...> class Graph,
          graph_attributes M, typename ...OptionalParams,
          typename Dom, typename MapFunc, typename Derived, typename ColorMap>
void color_graph(GView<Graph<DIRECTED, M, OptionalParams...>,
                 Dom, MapFunc, Derived> const& graph,
                 ColorMap& color_map)
{
  typedef Graph<UNDIRECTED, NONMULTIEDGES, OptionalParams...>
                                                        undirected_graph_type;
  typedef graph_view<undirected_graph_type,
                typename undirected_graph_type::domain_type,
                MapFunc,Derived>                        undirected_view_type;

  undirected_graph_type undir_graph;
  undirected_view_type undir_view(undir_graph);

  //Add vertices.
  map_func(graph_coloring_impl::build_vertex_undirected_graph_wf(),
           graph, make_repeat_view(undir_view));

  //Add undirected edges.
  map_func(graph_coloring_impl::build_edge_undirected_graph_wf(), graph,
           make_repeat_view(undir_view));

  undirected_view_type undir_view2(undir_graph);
  color_graph(undir_view2, color_map);
}


//////////////////////////////////////////////////////////////////////
/// @brief Parallel Level-Synchronized Graph Coloring Algorithm.
///
/// Assigns coloring to the vertices of the input graph, such that
/// neighboring vertices are not assigned the same color.
/// @param graph The @ref graph_view over the input graph.
/// @param color_map The output vertex property map where the color information
/// will be stored. [vertex->color (int)]
/// Specialization for @ref iterator_domain based @ref graph_view over
/// @ref dynamic_graph, because sparse-domain is needed in the algorithm to
/// have a view over the boundary vertices.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <template<typename, typename, typename, typename> class GView,
          template<graph_attributes, graph_attributes, typename...> class Graph,
          graph_attributes M, typename ...OptionalParams,
          typename Dist, typename DomMap,
          typename MapFunc, typename Derived, typename ColorMap>
void color_graph(GView<Graph<UNDIRECTED, M, OptionalParams...>,
                 iterator_domain<Dist, DomMap>, MapFunc, Derived>
                 const& graph, ColorMap& color_map)
{
  typedef Graph<UNDIRECTED, M, OptionalParams...>        undirected_graph_type;
  typedef GView<undirected_graph_type, domset1D<size_t>,
                MapFunc, Derived>                        view_type;
  typedef GView<undirected_graph_type,
                iterator_domain<Dist, DomMap>,
                MapFunc, Derived>                        original_view_type;

  //graph might have come from the directed color_graph method, which means
  //it might be invalid. The constructor below calls a map_reduce, so we need
  //to reconstruct the view in this case to make sure it is valid.
  original_view_type const& valid_graph = graph.is_valid() ?
    graph : original_view_type(
              graph.container(), graph.domain(), graph.mapfunc(), graph);

  view_type new_view(valid_graph);

  //Call graph coloring with the domset1D view
  color_graph(new_view, color_map);
}


//////////////////////////////////////////////////////////////////////
/// @brief Parallel Level-Synchronized Graph Coloring Algorithm.
///
/// Assigns coloring to the vertices of the input graph, such that
/// neighboring vertices are not assigned the same color.
/// @param graph The @ref graph_view over the input graph.
/// @param color_map The output vertex property map where the color information
/// will be stored. [vertex->color (int)]
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <typename GView, typename ColorMap>
void color_graph(GView const& graph, ColorMap& color_map)
{
  typedef GView                                       graph_view;
  typedef typename graph_view::gid_type               vertex_descriptor;
  typedef typename graph_view::domain_type            domain_type;
  typedef boost::unordered_map<vertex_descriptor,
                               size_t>                vertex_color_map_type;
  typedef boost::unordered_map<vertex_descriptor,
                               vertex_color_map_type> hashmap_type;
  typedef static_array<hashmap_type>                  array_type;
  typedef array_view<array_type>                      array_view_type;

  using namespace graph_coloring_impl;
  size_t num_locs = get_num_locations();
  array_type hashmap_array(num_locs);
  array_view_type hashmap_array_view(hashmap_array);

  map_func(graph_coloring_wf(), stapl::native_view(graph),
           make_repeat_view(graph), make_repeat_view(color_map),
           make_repeat_view(hashmap_array_view));

  /*Resolve conflicts with boundary vertices*/
  //create domains of boundary vertices
  typedef explicit_partition<typename GView::domain_type>   partition_type;
  typedef segmented_view<GView,
                           partition_type>                  segment_view_type;

  std::vector<domain_type> sub_domains =
    map_reduce(get_sub_domain<hashmap_type, domain_type>(),
               reduce_vector_wf<domain_type>(),hashmap_array_view);
  segment_view_type
    boundary_view(graph, partition_type(graph.domain(), sub_domains));

  while (map_reduce(resolve_color_conflicts(), plus<bool>(), boundary_view,
                    make_repeat_view(graph), make_repeat_view(color_map),
                    make_repeat_view(hashmap_array_view)))
  { }
}


//////////////////////////////////////////////////////////////////////
/// @brief Extracts the colors from the input graph, given the color map.
///
/// @param graph The @ref graph_view over the input graph.
/// @param color_map The input vertex property map where the color information
/// is stored. [vertex->color (int)]. Must have been previously populated by
/// calling @ref color_graph().
/// @return The domain of colors in the graph.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <typename GView, typename ColorMap>
domset1D<size_t> get_graph_colors(GView const& graph, ColorMap& color_map)
{
  typedef domset1D<size_t> domain_type;
  return map_reduce(graph_coloring_impl::get_vertex_color(),
                    plus<domain_type>(), graph, make_repeat_view(color_map));
}


//////////////////////////////////////////////////////////////////////
/// @brief Computes if the given coloring is valid for the input graph.
///
/// @param graph The @ref graph_view over the input graph.
/// @param color_map The input vertex property map where the color information
/// is stored. [vertex->color (int)]. Must have been previously populated by
/// calling @ref color_graph().
/// @return True if the coloring is valid, false otherwise.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <typename GView, typename ColorMap>
bool is_valid_graph_coloring(GView const& graph, ColorMap& color_map)
{
  return map_reduce(graph_coloring_impl::is_valid_graph_coloring_wf(),
                    logical_and<bool>(), graph, make_repeat_view(color_map));
}

} // namespace stapl

#endif
