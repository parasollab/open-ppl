/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_DYNAMIC_GRAPH_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_DYNAMIC_GRAPH_BASE_CONTAINER_HPP

#include <stapl/domains/iterator_domain.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>
#include <stapl/containers/sequential/graph/graph.h>
#include <stapl/containers/graph/base_container/base_container.hpp>
#include <stapl/containers/graph/base_container/vdg_hash_map.hpp>
#include <stapl/containers/type_traits/define_value_type.hpp>
#include <stapl/containers/graph/base_container/graph_storage.h>

namespace stapl {


//////////////////////////////////////////////////////////////////////
/// @brief Class for extracting descriptor from an iterator.
/// @ingroup pgraphBaseCont
//////////////////////////////////////////////////////////////////////
struct get_descriptor
{
  template <typename IT>
  typename IT::value_type::vertex_descriptor operator()(IT it) const
  {
    return (*it).descriptor();
  }
};

template <typename Traits>
struct dynamic_graph_base_container;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for
/// @ref dynamic_graph_base_container.
/// @ingroup pgraphTraitsBaseContainer
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
/// Must be default assignable, copyable and assignable.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, vertex type, edge type, edgelist type,
///  storage, etc. E.g. @ref graph_base_container_traits.
/// @todo Use inheritance of Traits idiom (see other base container)
///   to reduce code duplication (most of the types are already defined
///   there).
//////////////////////////////////////////////////////////////////////
template <typename Traits>
struct container_traits<dynamic_graph_base_container<Traits> >
{
private:
  typedef dynamic_graph_base_container<Traits> container_t;

public:
  typedef typename Traits::vertex_descriptor gid_type;
  typedef gid_type                           index_type;
  typedef typename Traits::vertex_impl_type  value_type;
  typedef typename Traits::domain_type       domain_type;
  typedef typename container_t::reference    reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief Traits class for @ref adjacency_list_model based on dynamic storage.
/// @ingroup pgraphTraitsStorage
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
/// Must be default assignable, copyable and assignable.
///
/// Uses @ref adjacency_list_hashmap_storage for vertices.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M,
          typename VertexP, typename EdgeP>
struct seq_graph_traits
{
  typedef typename define_value_type<VertexP>::type          stored_type;
  typedef stored_type                                        vertex_property;
  typedef EdgeP                                              edge_property;
  typedef size_t                                             vertex_descriptor;
  typedef size_t                                      simple_vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor>            edge_descriptor;
  typedef typename sequential::select_edge<
            vertex_descriptor, edge_property,D>::type        edge_type;
  typedef adjacency_list_impl<edge_type>                     edgelist_type;
  typedef vertex_adj_list_impl
          <vertex_descriptor,vertex_property,edgelist_type>  vertex_impl_type;
  typedef adjacency_list_hashmap_storage<seq_graph_traits>   storage_type;
  typedef typename sequential::graph_type<
                     seq_graph_traits,D>::type               directness_type;
  typedef typename sequential::graph_type<
                     seq_graph_traits,M>::type               multiplicity_type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Traits class for @ref dynamic_graph_base_container.
/// @ingroup pgraphTraitsBaseContainer
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
/// Must be default assignable, copyable and assignable.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M,
          typename VertexP, typename EdgeP>
struct dynamic_graph_base_container_traits
{
  typedef dynamic_graph_base_container_traits<
            D,M,VertexP,EdgeP>                               this_type;
public:
  typedef VertexP                                        raw_vertex_property;
  typedef VertexP                                            vertex_property;
  typedef typename define_value_type<VertexP>::type          stored_type;
  typedef EdgeP                                              edge_property;
  typedef size_t                                             vertex_descriptor;
  typedef size_t                                      simple_vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor>            edge_descriptor;
  typedef typename sequential::select_edge<
            vertex_descriptor, edge_property,D>::type        edge_type;

  typedef seq_graph_traits<D,M,VertexP,EdgeP>                seq_graph_traits_t;

  typedef typename seq_graph_traits_t::edgelist_type         edgelist_type;
  typedef typename seq_graph_traits_t::vertex_impl_type      vertex_impl_type;
  typedef adjacency_list_model<seq_graph_traits_t>           container_type;
  typedef typename sequential::graph_type<this_type,D>::type directness_type;
  typedef typename sequential::graph_type<this_type,M>::type multiplicity_type;
  static const graph_attributes d_type = D; //directedness
  static const graph_attributes m_type = M; //edge multiplicity

  typedef domset1D<simple_vertex_descriptor> domain_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for
/// @ref adjacency_list_model.
/// @ingroup pgraphTraitsBaseContainer
/// @tparam Traits A traits class for the @ref adjacency_list_model.
/// E.g. @ref seq_graph_traits.
//////////////////////////////////////////////////////////////////////
template <typename Traits>
struct container_traits<adjacency_list_model<Traits> >
{
  typedef typename Traits::vertex_descriptor  gid_type;
  typedef gid_type                            index_type;
  typedef typename Traits::vertex_impl_type   value_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Base-container for @ref dynamic_graph. Inherits all functionality
/// from @ref graph_base_container.
/// @ingroup pgraphBaseCont
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedgedness
/// (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
/// Must be default assignable, copyable and assignable.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, vertex type, edge type, edgelist type,
///  storage, etc. E.g. @ref dynamic_graph_base_container_traits.
//////////////////////////////////////////////////////////////////////
template <typename Traits>
struct dynamic_graph_base_container
  : public graph_base_container<Traits>
{
  typedef graph_base_container<Traits> base_type;

  typedef typename Traits::container_type             container_type;
  typedef typename Traits::domain_type                domain_type;
  typedef size_t                                      cid_type;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph base container with a given size, domain
  /// and id for the base-container.
  /// @param domain Provides the domain of vertex-descriptors stored in
  /// this base container.
  /// @param cid Provides the id of this base-container.
  //////////////////////////////////////////////////////////////////////
  template<typename ConstructDomainType>
  dynamic_graph_base_container(ConstructDomainType const& domain,
                               cid_type const& cid)
    : base_type(domain, cid)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph base container with a given size and domain,
  /// and constructs all elements with a default value for vertex.
  /// @param domain Provides the domain of vertex-descriptors stored in
  /// this base container.
  /// @param cid Provides the id of this base-container.
  /// @param default_value provides the default value for the vertices
  /// stored in this base-container.
  //////////////////////////////////////////////////////////////////////
  template<typename ConstructDomainType>
  dynamic_graph_base_container(ConstructDomainType const& domain,
                               cid_type const& cid,
                               typename base_type::value_type const&
                                 default_value)
    : base_type(domain, cid, default_value)
  { }

  dynamic_graph_base_container(dynamic_graph_base_container const& other)
    : base_type(other)
  { }

  domain_type domain(void) const
  {
    return this->m_domain;
  }
};

} // namespace stapl

#endif /*STAPL_CONTAINERS_DYNAMIC_GRAPH_BASE_CONTAINER_HPP*/
