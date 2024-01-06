/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_FWD_HPP
#define STAPL_CONTAINERS_GRAPH_FWD_HPP

#include <stapl/utility/use_default.hpp>

namespace stapl {

#ifdef STAPL_DOCUMENTATION_ONLY

//////////////////////////////////////////////////////////////////////
/// @brief Parallel static graph container. Inherits all functionality
/// from either @ref undirected_graph or @ref directed_graph.
/// @ingroup pgraph
///
/// Static graphs do not allow addition or deletion of vertices. The number of
/// vertices must be known at construction. Edges may be added/deleted.
/// Uses directedness selector to inherit from correct directed/undirected base.
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
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
/// default traits class is @ref static_graph_traits.
//////////////////////////////////////////////////////////////////////
template<graph_attributes D,
         graph_attributes M,
         typename VertexP  = use_default,
         typename EdgeP    = use_default,
         typename PS       = use_default,
         typename Map      = use_default,
         typename Traits   = use_default>
class graph;


//////////////////////////////////////////////////////////////////////
/// @brief Parallel dynamic graph container that supports addition
/// and deletion of vertices and edges.
/// @ingroup pgraphGeneral
///
/// Inherits from @ref stapl::graph and adds functionality to add/delete
/// vertices.
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam PS Partition strategy that defines how to partition
/// the original domain into subdomains. The default partition is
/// @ref balanced_partition over  an @ref indexed_domain.
/// @tparam Map Mapper that defines how to map the subdomains produced
/// by the partition to locations. The default mapper is @ref mapper.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, base container type, storage, etc. The
/// default traits class is @ref dynamic_graph_traits.
//////////////////////////////////////////////////////////////////////
template<graph_attributes D,
         graph_attributes M,
         typename VertexP  = use_default,
         typename EdgeP    = use_default,
         typename PS       = use_default,
         typename Map      = use_default,
         typename Traits   = use_default>
class dynamic_graph;

#else

template<graph_attributes D,
         graph_attributes M,
         typename ...OptionalParams>
class graph;

template<graph_attributes D,
         graph_attributes M,
         typename ...OptionalParams>
class dynamic_graph;

#endif // STAPL_DOCUMENTATION_ONLY

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_FWD_HPP
