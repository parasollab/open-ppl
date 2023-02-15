/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_HGRAPH_TRAITS_HPP
#define STAPL_CONTAINERS_HGRAPH_TRAITS_HPP

#include <stapl/containers/distribution/directory/map_manager.hpp>
#include <stapl/containers/distribution/directory/interval_registry.hpp>
#include <stapl/containers/distribution/directory/vector_directory.hpp>

#include <stapl/containers/distribution/container_manager/\
container_manager_dynamic_graph.hpp>
#include <stapl/containers/graph/base_container/hgraph_base_container.hpp>

#include <stapl/containers/mapping/mapper.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the @ref hierarchical_graph container.
/// Specifies customizable type parameters that could be changed on a
/// per-container basis.
/// @ingroup pgraphTraitsPGraph
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex.
/// @tparam EdgeP type of property for the edge.
/// @tparam PS Partition strategy that defines how to partition
/// the original domain into subdomains.
/// @tparam Map Mapper that defines how to map the subdomains produced
/// by the partition to locations.
////////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M,
         typename VertexP, typename EdgeP,
         typename PS, typename Map>
struct hgraph_traits
{
  typedef hgraph_traits<D, M, VertexP, EdgeP, PS, Map>        this_type;
  typedef VertexP                                             vertex_property;
  typedef EdgeP                                               edge_property;
  typedef size_t                                              vertex_descriptor;
  typedef size_t                                       simple_vertex_descriptor;
  typedef vertex_descriptor                                   gid_type;
  typedef edge_descriptor_impl<vertex_descriptor>             edge_descriptor;
  typedef typename sequential::graph_type<this_type,D>::type  directness_type;
  typedef typename sequential::graph_type<this_type,M>::type  multiplicity_type;
  typedef PS                                                  partition_type;
  typedef Map                                                 mapper_type;

  typedef map_manager<partition_type, mapper_type>            manager_type;

  typedef vector_directory<partition_type, mapper_type,
                           manager_type,
                           interval_registry<manager_type> >  directory_type;

  typedef hgraph_base_container_traits<
            D,M,VertexP,EdgeP>                       base_container_traits_type;
  typedef hgraph_base_container<D, M, VertexP, EdgeP>       base_container_type;

  typedef typename detail::construct_container_registry<
                     base_container_type, partition_type
                   >::type                             container_registry_type;

  typedef container_manager_dynamic_graph<base_container_type,
            container_registry_type>                    container_manager_type;
  typedef typename base_container_type::vertex_type           value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the distribution type based on a
  /// container type.
  /// @tparam C Type of the container.
  //////////////////////////////////////////////////////////////////////
  template <typename C>
  struct construct_distribution
  {
    typedef graph_distribution<C> type;
  };

};

} // namespace stapl

#endif
