/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_OUT_OF_CORE_GRAPH_HPP
#define STAPL_CONTAINERS_OUT_OF_CORE_GRAPH_HPP

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/sequential/graph/graph_util.h>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/traits/storage_graph_traits.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/containers/distribution/is_distribution_view.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>

#include <stapl/utility/use_default.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Class for selecting between default and optional template parameters
/// for @ref out_of_core_graph.
/// @ingroup pgraphImpl
/// @tparam D graph-attribute specifying Directedness. (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP (Optional) type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP (Optional) type of property for the edge.
/// Must be default assignable, copyable and assignable.
/// @tparam PS (Optional) Partition strategy that defines how to partition
/// the original domain into subdomains.
/// @tparam M (Optional) Mapper that defines how to map the subdomains produced
/// by the partition to locations.
/// @tparam Traits (Optional) A traits class that defines customizable
/// components of graph, such as the domain type, base container type, storage,
/// etc.
//////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M, typename ...OptionalParams>
struct storage_graph_param_selector
{
  typedef tuple<
    properties::no_property,
    properties::no_property,
    view_based_partition<distribution_spec<>>, // Partition
    view_based_mapper<distribution_spec<>>,    // Mapper
    storage_graph_traits<D, M, properties::no_property, properties::no_property,
                         balanced_partition<indexed_domain<size_t> >,
                         mapper<size_t> >
    >                                                   default_value_types;

  typedef typename compute_type_parameters<
    default_value_types, OptionalParams...
  >::type                                               param_types;


  typedef typename tuple_element<0, param_types>::type  vertex_prop;
  typedef typename tuple_element<1, param_types>::type  edge_prop;
  typedef typename tuple_element<2, param_types>::type  partition_t;
  typedef typename tuple_element<3, param_types>::type  mapper_t;

  typedef tuple<
    vertex_prop, edge_prop, partition_t, mapper_t,
    storage_graph_traits<D, M, vertex_prop, edge_prop, partition_t, mapper_t>
    >                                                   default_traits_type;

  typedef typename compute_type_parameters<
    default_traits_type, OptionalParams...
  >::type                                               traits_param_type;

  typedef typename tuple_element<4, traits_param_type>::type  traits_t;
};


//////////////////////////////////////////////////////////////////////
/// @brief Creates an Out-of-Core graph.
/// @ingroup pgraphSpecial
///
/// Out-of-Core graphs can be processed from disk, and may not be completely
/// in RAM. The number of vertices must be known at construction. Edges may
/// be added/deleted.
/// @tparam VertexP type of property for the vertex. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam PS Partition strategy that defines how to partition
/// the original domain into subdomains. The default partition is
/// @ref balanced_partition.
/// @tparam Map Mapper that defines how to map the subdomains produced
/// by the partition to locations. The default mapper is @ref mapper.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, base container type, storage, etc. The
/// default traits class is @ref storage_graph_traits.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M, typename ...OptionalParams>
class out_of_core_graph
  : public graph<D, M,
  typename storage_graph_param_selector<D, M, OptionalParams...>::vertex_prop,
  typename storage_graph_param_selector<D, M, OptionalParams...>::edge_prop,
  typename storage_graph_param_selector<D, M, OptionalParams...>::partition_t,
  typename storage_graph_param_selector<D, M, OptionalParams...>::mapper_t,
  typename storage_graph_param_selector<D, M, OptionalParams...>::traits_t>
{
protected:

  typedef typename storage_graph_param_selector<
    D, M, OptionalParams...>::vertex_prop        VertexP;
  typedef typename storage_graph_param_selector<
    D, M, OptionalParams...>::edge_prop          EdgeP;
  typedef typename storage_graph_param_selector<
    D, M, OptionalParams...>::partition_t        PS;
  typedef typename storage_graph_param_selector<
    D, M, OptionalParams...>::mapper_t           Map;
  typedef typename storage_graph_param_selector<
    D, M, OptionalParams...>::traits_t           Traits;

  typedef graph<D, M, VertexP, EdgeP, PS, Map, Traits>       base_type;
  typedef out_of_core_graph<D, M, OptionalParams...>         this_type;
  typedef typename graph_directedness_container_selector<
    D, M, VertexP, EdgeP, PS, Map, Traits>::type             directed_base_type;

public:
  STAPL_IMPORT_TYPE(typename base_type, size_type)
  STAPL_IMPORT_TYPE(typename base_type, partition_type)
  STAPL_IMPORT_TYPE(typename base_type, mapper_type)
  STAPL_IMPORT_TYPE(typename base_type, distribution_type)
  STAPL_IMPORT_TYPE(typename base_type, value_type)
  STAPL_IMPORT_TYPE(typename base_type, vertex_descriptor)
  STAPL_IMPORT_TYPE(typename base_type, edge_descriptor)
  STAPL_IMPORT_TYPE(typename base_type, vertex_property)
  STAPL_IMPORT_TYPE(typename base_type, edge_property)

public:

  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(size_t const&, vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  out_of_core_graph(size_t const& n=0,
          typename base_type::vertex_property const& default_value
            =typename base_type::vertex_property())
    : base_type(n, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(size_t const&, vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  out_of_core_graph(size_t const& n, size_t const& block_size,
          typename base_type::vertex_property const& default_value
            =typename base_type::vertex_property())
    : base_type(block_cyclic(n, block_size), default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(partition_type const&, vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  out_of_core_graph(typename base_type::partition_type const& ps,
          typename base_type::vertex_property const& default_value
            =typename base_type::vertex_property())
    : base_type(ps, default_value)
  { }

  boost::shared_ptr<this_type> shared_from_this()
  {
    return boost::static_pointer_cast<this_type>(
             boost::enable_shared_from_this<detail::container_impl<
               directed_base_type>>::shared_from_this());
  }
  /// @}
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for
/// @ref stapl::out_of_core_graph.
/// @ingroup pgraphTraits
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexPx type of property for the vertex. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgePx type of property for the edge. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam PSx Partition strategy that defines how to partition
/// the original domain into subdomains. The default partition is
/// @ref balanced_partition.
/// @tparam Mapx Mapper that defines how to map the subdomains produced
/// by the partition to locations. The default mapper is @ref mapper.
//////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M,
         typename ...OptionalParams>
struct container_traits<out_of_core_graph<D,M,OptionalParams...> >
  : graph_param_selector<D,M, OptionalParams...>::traits_t
{
  typedef out_of_core_graph<D,M,OptionalParams...>     container_t;

  typedef typename graph_param_selector<
    D, M, OptionalParams...>::partition_t              partition_t;
  typedef typename graph_param_selector<
    D, M, OptionalParams...>::mapper_t                 mapper_t;
  typedef typename graph_param_selector<
    D, M, OptionalParams...>::traits_t                 traits_t;

  typedef typename container_t::value_type             value_type;
  typedef typename container_t::domain_type            domain_type;

  typedef typename traits_t::
    template construct_distribution<container_t>::type dist_t;
  typedef graph_accessor<dist_t>                       accessor_type;
  typedef proxy<value_type, accessor_type>             reference;
};

} // stapl namespace


#endif /* STAPL_CONTAINERS_OUT_OF_CORE_GRAPH_HPP */
