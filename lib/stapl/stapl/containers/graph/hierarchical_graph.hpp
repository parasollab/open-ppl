/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_HIERARCHICAL_GRAPH_HPP
#define STAPL_CONTAINERS_HIERARCHICAL_GRAPH_HPP

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/traits/hgraph_traits.hpp>
#include <stapl/domains/domain_interval.hpp>
#include <stapl/utility/use_default.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Class for selecting between default and optional template parameters
/// for @ref hierarchical_graph.
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
struct hgraph_param_selector
{
  typedef tuple<
    properties::no_property,
    properties::no_property,
    balanced_partition<indexed_domain<size_t> >, // Partition
    mapper<size_t>,                              // Mapper
    hgraph_traits<D, M, properties::no_property, properties::no_property,
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
    hgraph_traits<D, M, vertex_prop, edge_prop, partition_t, mapper_t>
    >                                                   default_traits_type;

  typedef typename compute_type_parameters<
    default_traits_type, OptionalParams...
  >::type                                               traits_param_type;

  typedef typename tuple_element<4, traits_param_type>::type  traits_t;
};


//////////////////////////////////////////////////////////////////////
/// @brief Hierarchical graphs are dynamic graphs that support hierarchies
/// of supervertices. Each vertex is associated with its children in the
/// level below. Dynamic graphs support addition and deletion of vertices
/// and edges.
/// @ingroup pgraphGeneral
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexPx type of property for the vertex. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgePx type of property for the edge. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam PSx Partition strategy that defines how to partition
/// the original domain into subdomains. The default partition is
/// @ref balanced_partition over  an @ref indexed_domain.
/// @tparam Mapx Mapper that defines how to map the subdomains produced
/// by the partition to locations. The default mapper is @ref mapper.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, base container type, storage, etc. The
/// default traits class is @ref hgraph_traits.
//////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M,
         typename ...OptionalParams>
class hierarchical_graph
  : public dynamic_graph<D, M,
  typename hgraph_param_selector<D, M, OptionalParams...>::vertex_prop,
  typename hgraph_param_selector<D, M, OptionalParams...>::edge_prop,
  typename hgraph_param_selector<D, M, OptionalParams...>::partition_t,
  typename hgraph_param_selector<D, M, OptionalParams...>::mapper_t,
  typename hgraph_param_selector<D, M, OptionalParams...>::traits_t>
{
private:

  typedef typename hgraph_param_selector<
    D, M, OptionalParams...>::vertex_prop        VertexP;
  typedef typename hgraph_param_selector<
    D, M, OptionalParams...>::edge_prop          EdgeP;
  typedef typename hgraph_param_selector<
    D, M, OptionalParams...>::partition_t        PS;
  typedef typename hgraph_param_selector<
    D, M, OptionalParams...>::mapper_t           Map;
  typedef typename hgraph_param_selector<
    D, M, OptionalParams...>::traits_t           Traits;

  typedef dynamic_graph<D, M, VertexP, EdgeP, PS, Map, Traits> base_type;

public:
  typedef typename base_type::partition_type                partition_type;
  typedef typename base_type::distribution_type             distribution_type;
  typedef domainset1D<distribution_type>                    domain_type;

  typedef typename base_type::vertex_property               vertex_property;

  /// @name Constructors
  /// @{

  hierarchical_graph()
    : base_type()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(size_t const&, vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  hierarchical_graph(size_t const& n, VertexP const& default_value=VertexP())
    : base_type(n, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(partition_type const&, vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  hierarchical_graph(partition_type const& ps,
                     VertexP const& default_value=VertexP())
    : base_type(ps, default_value)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(size_t, vertex_property const&, DP const&)
  //////////////////////////////////////////////////////////////////////
  template <typename DP>
  hierarchical_graph(size_t const& n, vertex_property const& default_value,
                     DP const& dis_policy)
    : base_type(n, default_value, dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(boost::tuples::cons<X,Y>)
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y>
  hierarchical_graph(boost::tuples::cons<X,Y> dims)
    : base_type(dims)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(boost::tuples::cons<X,Y>, DP const&)
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y, typename DP>
  hierarchical_graph(boost::tuples::cons<X,Y> dims, const DP& dis_policy)
    : base_type(dims, dis_policy)
  { }

  /// @}

  /// @name Memory and Domain Management
  /// @{

  domain_type domain(void) const
  {
    return domain_type(this->distribution().domain());
  }

  /// @}
};


} // stapl namespace

#endif /* STAPL_CONTAINERS_HIERARCHICAL_GRAPH_HPP */
