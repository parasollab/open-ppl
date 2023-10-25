/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_REBALANCE_DIFFUSIVE_HPP
#define STAPL_CONTAINERS_GRAPH_REBALANCE_DIFFUSIVE_HPP

#include <stapl/runtime.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/containers/graph/partitioners/gpartition.h>
#include <stapl/containers/graph/partitioners/graph_partitioners.h>
#include <stapl/containers/graph/partitioners/refiners.hpp>
#include <stapl/containers/generators/single.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/counting_view.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief A function object used to give a default weight of 1 for
/// vertices when rebalancing an unweighted graph.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct return_1
{
  typedef size_t value_type;

  template<typename T>
  value_type get(T const&) const
  {
    return 1;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A function object that is effectively a no-op, which is used
/// as the default action when rebalancing a graph that does not need
/// any specialized functionality.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct do_nothing
{
  typedef void result_type;

  template<typename T, typename U, typename V>
  result_type operator()(T, U, V) const
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compute a 2-vector of independent sets, where each independent
/// set is a pair of adjacent vertices.
/// @ingroup pgraphAlgoDetails
///
/// For example, consider the linear GID space of [0, ..., n-1]. The
/// computed independent sets would be one vector that contains adjacent
/// vertices starting at 0:
///   [(0, 1), (2, 3), ..., (n-1, n)]
/// and another vector that starts at 1:
///   [(1, 2), (3, 4), ..., (n-2, n-1)]
///
/// @param x Size of the linear chain of vertices
//////////////////////////////////////////////////////////////////////
std::vector<std::vector<std::pair<size_t, size_t> > >
linear_independent_sets(size_t const& x)
{
  std::vector<std::vector<std::pair<size_t, size_t> > > is;

  // group adjacent vertices starting at 0
  std::vector<std::pair<size_t, size_t> > edges;
  for (size_t i = 0; i+1 < x; i+=2)
    edges.push_back(std::make_pair(i, i+1));

  if (!edges.empty())
    is.push_back(edges);

  // group adjacent vertices starting at 1
  edges.clear();
  for (size_t i = 1; i+1 < x; i+=2)
    edges.push_back(std::make_pair(i, i+1));

  if (!edges.empty())
    is.push_back(edges);

  return is;
}


//////////////////////////////////////////////////////////////////////
/// @brief Function object that is used to migrate a single vertex on
/// the source location, first executing the migration function if any.
/// @ingroup pgraphAlgoDetails
///
/// @tparam GID     The GID type of the graph
/// @tparam G       Graph type
/// @tparam Functor Migration action function
//////////////////////////////////////////////////////////////////////
template<typename Functor, typename GID, typename G>
class migrator
: private Functor
{
private:
  GID                         m_gid;
  location_type               m_loc;
  p_object_pointer_wrapper<G> m_g;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create a migrator functor for a given GID, with a destination
  /// location, source view and a migration action function.
  /// @param g GID of the vertex to migrate
  /// @param location Location to which to migrate the vertex
  /// @param v View of the graph
  /// @param f Function to execute before migrating the vertex. The expected
  /// signature is operator()(value_type, gid_type, location_type);
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  migrator(F&& f, GID const& gid, location_type location, G& g)
    : Functor(std::forward<F>(f)),
      m_gid(gid),
      m_loc(location),
      m_g(&g)
  { }

  template<typename T>
  void operator()(T t) const
  {
    static_cast<Functor const&>(*this)(t, m_gid, m_loc);
    m_g->migrate(m_gid, m_loc);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Serialization for this function object.
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.base<Functor>(*this);
    t.member(m_gid);
    t.member(m_loc);
    t.member(m_g);
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Function object that is used to compute the difference between
/// the native partition and a desired partition.
/// @ingroup pgraphAlgoDetails
///
/// @tparam P Graph partition to achieve
/// @tparam M A map from GIDs to action functions
//////////////////////////////////////////////////////////////////////
template<typename P, typename M>
class compute_differences
{
  typedef typename M::mapped_type action_function_type;
  P m_part;
  M m_action_map;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Instantiate the function object with a desired partition
  /// and an action function map.
  /// @param p The desired partition. This partition must have exactly
  /// as many partitions as locations.
  /// @param action_map A map from GIDs to action functions
  //////////////////////////////////////////////////////////////////////
  compute_differences(P const& p, M const& action_map)
    : m_part(p), m_action_map(action_map)
  { }

  template<typename T, typename G>
  void operator()(T loc, G g)
  {
    typename P::partition_view_t::vertex_property vw =
      m_part.partition()[loc].property();
    typename P::partition_view_t::vertex_property::domain_type dom =
      vw.domain();

    auto& graph = g.container();

    for (size_t i = dom.first(); i != dom.open_last(); i = dom.advance(i, 1))
    {
      if (!graph.distribution().container_manager().contains(i))
      {
        typedef detail::migrator<
                  action_function_type,
                  size_t,
                  typename view_traits<G>::container
                > wf_type;
        g.apply_set(i, wf_type{m_action_map[i], i, loc, graph});
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Serialization for this function object.
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.member(m_part);
    t.member(m_action_map);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Rebalance a graph diffusively by iteratively moving vertices
/// between neighboring partitions.
/// @ingroup pgraphAlgo
///
/// Balances the graph based on vertex-weights by migrating vertices to
/// create an even weight-distribution among the locations, while attempting
/// to minimize total movement of vertices.
///
/// @param v The @ref graph_view over the input graph.
/// @param weight_map The vertex property map storing weights for each vertex.
/// @param migration_action_map A map from vertex descriptor to functor
/// describing an action to perform when a vertex is migrated. This map
/// must export an interface compatible with std::map, where the function
/// to execute has the signature (value_type, gid_type, location_type)
/// @param independent_sets A collection of independent sets, where each
/// independent set is represented as a vector of edges (pairs of vertices).
/// An example independent set that can be used is
/// @ref linear_independent_sets()
///
/// @warning Invalidates the input graph_view.
//////////////////////////////////////////////////////////////////////
template<typename C, typename W, typename M, typename IS>
void rebalance_diffusive(C& v, W const& weight_map,
                         M const& migration_action_map,
                         IS const& independent_sets)
{
  typedef gpartition<C> partition_type;

  // create native partition
  native_partitioner<C> np;
  native_ef ef;
  partition_type native_part(create_level(v, np, ef));

  partition_type part = graph_repartition(native_part,
    weight_balanced_refiner<W>(weight_map, independent_sets, 0));

  map_func(compute_differences<partition_type, M>(part, migration_action_map),
           counting_view<location_type>(get_num_locations()),
           make_repeat_view(v));
};


//////////////////////////////////////////////////////////////////////
/// @brief Rebalance a graph diffusively by iteratively moving vertices
/// between neighboring partitions.
///
/// Balances the graph based on vertex-weights by migrating vertices to
/// create an even weight-distribution among the locations, while attempting
/// to minimize total movement of vertices.
///
/// @param v The @ref graph_view over the input graph.
/// @param weight_map The vertex property map storing weights for each vertex.
/// @warning Invalidates the input graph_view.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename C, typename W>
void rebalance_diffusive(C& v, W const& weight_map)
{
  typedef single_container<detail::do_nothing> migration_action_map_type;

  detail::do_nothing dn;
  migration_action_map_type action_function(dn);

  std::vector<std::vector<std::pair<size_t, size_t> > > independent_sets =
    detail::linear_independent_sets(get_num_locations());

  rebalance_diffusive(v, weight_map, action_function, independent_sets);
}


//////////////////////////////////////////////////////////////////////
/// @brief Rebalance a graph diffusively by iteratively moving vertices
/// between neighboring partitions.
///
/// Balances the graph based on vertex-weights by migrating vertices to
/// create an even weight-distribution among the locations, while attempting
/// to minimize total movement of vertices.
///
/// @param v The @ref graph_view over the input graph.
/// @warning Invalidates the input graph_view.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename C>
void rebalance_diffusive(C& v)
{
  detail::return_1 r;
  graph_internal_property_map<C, detail::return_1> weight(v, r);

  rebalance_diffusive(v, weight);
};

} // namespace stapl

#endif
