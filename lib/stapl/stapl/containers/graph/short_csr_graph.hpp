/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SHORT_CSR_GRAPH_HPP
#define STAPL_CONTAINERS_SHORT_CSR_GRAPH_HPP

#include <stapl/containers/graph/csr_graph.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>

#include <cstdint>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Base container traits for a CSR with short edges.
/// @ingroup pgraphTraitsBaseContainer
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam VertexP type of property for the vertex.
/// @tparam EdgeP type of property for the edge.
/// @tparam Dom Domain type of the base container.
////////////////////////////////////////////////////////////////////////
template<graph_attributes D, typename VertexP, typename EdgeP, typename Dom>
class csr_short_edge_base_container_traits
{
  using this_type = csr_short_edge_base_container_traits;

public:
  using raw_vertex_property = VertexP;
  using stored_type = typename define_value_type<VertexP>::type;
  using vertex_property = stored_type;
  using edge_property = EdgeP;
  using vertex_descriptor = typename Dom::index_type;
  using simple_vertex_descriptor = typename Dom::index_type;
  using edge_descriptor = edge_descriptor_impl<vertex_descriptor>;
  using full_edge_type = typename sequential::select_edge<
    vertex_descriptor, edge_property, D
  >::type;

  using edge_type =
    typename sequential::compute_short_edge_type<full_edge_type>::type;

  using edgelist_type = csr_edgelist_impl<edge_type>;

  using vertex_impl_type = vertex_csr_impl<
    vertex_descriptor, vertex_property, edgelist_type
  >;

  using storage_type = csr_vector_storage<this_type>;
  using container_type = csr_model<this_type>;
  using domain_type = Dom;

  static const graph_attributes m_type = NONMULTIEDGES;
  static const graph_attributes d_type = D; //directedness

  using directness_type = typename sequential::graph_type<this_type,D>::type;
  using multiplicity_type =
    typename sequential::graph_type<this_type, m_type>::type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Traits for the short_csr_graph.
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam VertexP type of property for the vertex.
/// @tparam EdgeP type of property for the edge.
/// @tparam PS Partition strategy that defines how to partition
/// the original domain into subdomains.
/// @tparam Map Mapper that defines how to map the subdomains produced
/// by the partition to locations.
////////////////////////////////////////////////////////////////////////
template<graph_attributes D,
         typename VertexP, typename EdgeP,
         typename PS, typename Map>
struct csr_short_edge_graph_traits
  : csr_graph_base_traits<
      D, NONMULTIEDGES, VertexP, EdgeP, PS, Map,
      csr_short_edge_base_container_traits<
        D, VertexP, EdgeP, typename PS::value_type
      >
    >
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Type alias for a CSR (compressed sparse row) graph that has
///        short edges. A short edge is an edge that only has target
///        information and no source or ID information in the case of
///        multiedges.
///
/// @see csr_graph
///
/// @note By definition, a graph with short edges cannot be multiedged,
///       as there is no ID to distinguish between multiple edges to the
///       same source-target pair.
///
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam VP Type of property for the vertex. Default is no_property.
/// @tparam EP Type of property for the edge. Default is no_property.
/// @tparam P (Optional) Partition strategy that defines how to partition
/// the original domain into subdomains.
/// @tparam M (Optional) Mapper that defines how to map the subdomains produced
/// by the partition to locations.
//////////////////////////////////////////////////////////////////////
template<graph_attributes D,
         typename VP = properties::no_property,
         typename EP = properties::no_property,
         typename P = balanced_partition<indexed_domain<std::size_t>>,
         typename M = mapper<std::size_t>>
using short_csr_graph = csr_graph<
  D, NONMULTIEDGES, VP, EP, P, M,
  csr_short_edge_graph_traits<
    D, VP, EP, P, M
  >
>;

//////////////////////////////////////////////////////////////////////
/// @brief Type alias for a CSR (compressed sparse row) graph that has
///        short edges and is limited to 2^32 vertices. A short edge is
///        an edge that only has target information and no source or ID
///        information in the case of multiedges.
///
///        In terms of memory, this is the smallest graph that SGL offers
///        with storage requirement of $$8n+4m$$ bytes, in addition to other
///        auxiliary metadata and properties.
///
/// @see csr_graph
///
/// @note By definition, a graph with short edges cannot be multiedged,
///       as there is no ID to distinguish between multiple edges to the
///       same source-target pair.
///
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam VP Type of property for the vertex. Default is no_property.
/// @tparam EP Type of property for the edge. Default is no_property.
/// @tparam P (Optional) Partition strategy that defines how to partition
/// the original domain into subdomains.
/// @tparam M (Optional) Mapper that defines how to map the subdomains produced
/// by the partition to locations.
//////////////////////////////////////////////////////////////////////
template<graph_attributes D,
         typename VP = properties::no_property,
         typename EP = properties::no_property,
         typename P = balanced_partition<indexed_domain<std::uint32_t>>,
         typename M = mapper<std::uint32_t>>
using small_short_csr_graph = csr_graph<
  D, NONMULTIEDGES, VP, EP, P, M,
  csr_short_edge_graph_traits<
    D, VP, EP, P, M
  >
>;

} // stapl namespace

#endif
