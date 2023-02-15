/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CSR_GRAPH_HPP
#define STAPL_CONTAINERS_CSR_GRAPH_HPP

#include <stapl/containers/sequential/graph/graph_util.h>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/traits/csr_graph_traits.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/domains/indexed.hpp>

#include <stapl/utility/use_default.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Implementation for a Compressed Sparse-Row (CSR) graph.
/// @ingroup pgraphSpecial
///
/// CSR graphs do not allow addition and deletion of vertices or edges once
/// the graph has been "committed". All edges must be added before calling
/// the commit() method, which finalizes the graph.
/// Inherits all methods from @ref stapl::graph and adds method to commit.
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
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, base container type, storage, etc. The
/// default traits class is @ref csr_graph_traits.
//////////////////////////////////////////////////////////////////////
/*
template <graph_attributes D, graph_attributes M,
          typename VertexP = properties::no_property,
          typename EdgeP   = properties::no_property,
          typename PS      = balanced_partition< indexed_domain<size_t> > ,
          typename Map     = mapper<size_t>,
          typename Traits  = csr_graph_traits<D, M, VertexP, EdgeP, PS, Map> >
*/
template<graph_attributes D, graph_attributes M,
         typename VertexPx = use_default, typename EdgePx = use_default,
         typename PSx = use_default, typename Mapx = use_default,
         typename Traitsx = use_default>
class csr_graph
  : public graph<D, M,
                 typename select_parameter<VertexPx,
                                           properties::no_property>::type,
                 typename select_parameter<EdgePx,
                                           properties::no_property>::type,
                 typename select_parameter<PSx,
                   balanced_partition<indexed_domain<size_t> > >::type,
                 typename select_parameter<Mapx, mapper<size_t> >::type,
                 typename select_parameter<Traitsx,
                   csr_graph_traits<D, M,
                     typename select_parameter<VertexPx,
                                               properties::no_property>::type,
                     typename select_parameter<EdgePx,
                                               properties::no_property>::type,
                     typename select_parameter<PSx,
                       balanced_partition<indexed_domain<size_t> > >::type,
                     typename select_parameter<Mapx, mapper<size_t> >::type>
                   >::type>
{
private:
  typedef typename select_parameter<VertexPx,
                                    properties::no_property>::type VertexP;
  typedef typename select_parameter<EdgePx,
                                    properties::no_property>::type EdgeP;
  typedef typename select_parameter<PSx,
    balanced_partition<indexed_domain<size_t> > >::type           PS;
  typedef typename select_parameter<Mapx, mapper<size_t> >::type  Map;
  typedef typename select_parameter<Traitsx,
                                    csr_graph_traits<D, M,
                                                     VertexP, EdgeP,
                                                     PS, Map>
                                    >::type                      traits_type;

  typedef csr_graph<D, M, VertexP, EdgeP, PS, Map, traits_type>  this_type;
  typedef graph<D, M, VertexP, EdgeP, PS, Map, traits_type>      base_type;
  typedef typename graph_directedness_container_selector<
    D, M, VertexP, EdgeP, PS, Map, traits_type>::type       directed_base_type;

public:
  typedef typename base_type::partition_type                partition_type;
  typedef typename base_type::mapper_type                   mapper_type;
  typedef typename base_type::distribution_type             distribution_type;
  typedef typename base_type::vertex_descriptor             vertex_descriptor;
  typedef typename partition_type::domain_type         descriptor_domain_type;
  typedef typename Map::domain_type                         map_dom_t;

  typedef typename base_type::vertex_property               vertex_property;

  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(size_t const&, vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  csr_graph(size_t const& n, VertexP const& default_value=VertexP())
  : base_type(n, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(partition_type const&, vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  csr_graph(partition_type const& ps, VertexP const& default_value=VertexP())
    : base_type(ps, default_value)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(size_t, vertex_property const&, DP const&)
  //////////////////////////////////////////////////////////////////////
  template <typename DP>
  csr_graph(size_t n, vertex_property const& default_value,
            DP const& dis_policy)
    : base_type(n, default_value, dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(boost::tuples::cons<X,Y>)
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y>
  csr_graph(boost::tuples::cons<X,Y> dims)
    : base_type(dims)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::graph(boost::tuples::cons<X,Y>, DP const&)
  //////////////////////////////////////////////////////////////////////
  template <typename X, typename Y, typename DP>
  csr_graph(boost::tuples::cons<X,Y> dims, DP const& dis_policy)
    : base_type(dims, dis_policy)
  { }

  /// @}

  /// @name Memory and Domain Management
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Commit the CSR graph.
  ///
  /// All edges must have been added before calling this method.
  /// No further addition of edges is possible after this call until
  /// an uncommit is called.
  //////////////////////////////////////////////////////////////////////
  inline void commit()
  {
    for (auto& bc : this->distribution().container_manager())
      bc.container().commit();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Uncommit the CSR graph.
  ///
  /// Place the graph in a state so that edges can be added. While the graph
  /// is uncommited, it is not possible to access its edges.
  //////////////////////////////////////////////////////////////////////
  inline void uncommit()
  {
    for (auto& bc : this->distribution().container_manager())
      bc.container().uncommit();
  }

  boost::shared_ptr<this_type> shared_from_this()
  {
    return boost::static_pointer_cast<this_type>(
             boost::enable_shared_from_this<detail::container_impl<
               directed_base_type>>::shared_from_this());
  }
  /// @}
};


} // stapl namespace

#endif /* STAPL_CONTAINERS_CSR_GRAPH_HPP */
