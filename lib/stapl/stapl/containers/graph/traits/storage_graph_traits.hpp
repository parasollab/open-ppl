/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_STORAGE_GRAPH_TRAITS_HPP
#define STAPL_CONTAINERS_GRAPH_STORAGE_GRAPH_TRAITS_HPP

#include <stapl/containers/distribution/container_manager/container_manager_static_graph.hpp>
#include <stapl/containers/distribution/container_manager/registry/single.hpp>
#include <stapl/containers/distribution/container_manager/registry/interval.hpp>
#include <stapl/containers/distribution/container_manager/registry/interval_disk.hpp>
#include <stapl/containers/distribution/directory/container_directory.hpp>
#include <stapl/containers/distribution/directory/static_registry.hpp>
#include <stapl/containers/graph/distribution_static.hpp>
#include <stapl/containers/graph/base_container/base_container_storage.hpp>
#include <stapl/containers/mapping/mapper.hpp>

namespace stapl {

namespace detail {

class graph_nested_initializer;

template<typename BaseContainer, typename Partition>
struct construct_storage_container_registry
{
  typedef interval_disk_container_registry<BaseContainer> type;
};

template<typename BaseContainer, typename Domain>
struct construct_storage_container_registry<BaseContainer,
                                            balanced_partition<Domain> >
{
  typedef single_container_registry<BaseContainer> type;
};


} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the @ref stapl::graph container.
/// Specifies customizable type parameters that could be changed on a
/// per-container basis.
/// @ingroup pgraphTraitsPGraph
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexPx type of property for the vertex.
/// @tparam EdgePx type of property for the edge.
/// @tparam PS Partition strategy that defines how to partition
/// the original domain into subdomains.
/// @tparam Map Mapper that defines how to map the subdomains produced
/// by the partition to locations.
////////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M,
         typename VertexP, typename EdgeP,
         typename PS, typename Map>
struct storage_graph_traits
{
  typedef storage_graph_traits<D, M, VertexP, EdgeP, PS, Map> this_type;
  typedef VertexP                                      vertex_property;
  typedef EdgeP                                        edge_property;
  typedef size_t                                       vertex_descriptor;
  typedef size_t                                       simple_vertex_descriptor;
  typedef vertex_descriptor                            gid_type;
  typedef edge_descriptor_impl<vertex_descriptor>      edge_descriptor;
  typedef typename sequential::graph_type<this_type,D>::type  directness_type;
  typedef typename sequential::graph_type<this_type,M>::type  multiplicity_type;
  typedef PS                                                  partition_type;
  typedef Map                                                 mapper_type;
  typedef typename PS::value_type                             domain_type;
  typedef typename domain_type::index_type                    index_type;
  typedef typename PS::index_type                             cid_type;
  typedef typename domain_type::size_type                     size_type;


  typedef graph_base_container_storage<
    graph_base_container_traits<D,M,VertexP,EdgeP, domain_type> >
                                                       base_container_type;

  typedef typename detail::construct_storage_container_registry<
                     base_container_type, partition_type
                   >::type                             container_registry_type;

  typedef container_manager_static_graph<base_container_type,
    container_registry_type>
                                                         container_manager_type;

  typedef typename base_container_type::vertex_type      value_type;

protected:
  typedef directory_impl::manager<partition_type, mapper_type> manager_type;

public:
  typedef container_directory<
            partition_type,
            mapper_type,
            manager_type,
            static_registry<manager_type>
          >                                              directory_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the distribution type based on a
  /// container type.
  /// @tparam C Type of the container.
  //////////////////////////////////////////////////////////////////////
  template <typename C>
  struct construct_distribution
  {
    typedef graph_distribution_static<C>   type;
  };

  typedef void enable_view_reference;
  template <typename C>
  struct construct_view
  {
    typedef graph_view<C> type;
  };

  typedef vertex_property                  nested_stored_value_type;
  typedef detail::graph_nested_initializer nested_initializer_type;
};

} // namespace stapl

#endif
