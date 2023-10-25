/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_HGRAPH_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_GRAPH_HGRAPH_BASE_CONTAINER_HPP

#include <stapl/containers/graph/base_container/dynamic_base_container.hpp>
#include <stapl/domains/interval.hpp>
#include <stapl/containers/graph/base_container/hgraph_storage.h>

namespace stapl {

template <graph_attributes D,
          graph_attributes M,
          typename VertexP,
          typename EdgeP>
struct hgraph_base_container;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for
/// @ref hgraph_base_container.
/// @ingroup pgraphTraitsBaseContainer
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex.
///   Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
///   Must be default assignable, copyable and assignable.
/// @todo Use inheritance of Traits idiom (see other base container)
///   to reduce code duplication (most of the types are already defined
///   there).
//////////////////////////////////////////////////////////////////////
template <graph_attributes D,
          graph_attributes M,
          typename VertexP,
          typename EdgeP>
struct container_traits<hgraph_base_container<D, M, VertexP, EdgeP> >
{
  typedef hgraph_base_container<D, M, VertexP, EdgeP> base_type;
  typedef typename base_type::traits_type             traits_type;
  typedef typename traits_type::vertex_descriptor     gid_type;
  typedef gid_type                                    index_type;
  typedef typename traits_type::vertex_impl_type      value_type;
  typedef typename traits_type::domain_type           domain_type;
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
/// Provides special vertex for hierarchical graphs.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M, typename VertexP,
          typename EdgeP>
struct hgraph_seq_graph_traits
{
  typedef typename define_value_type<VertexP>::type          stored_type;
  typedef stored_type                                        vertex_property;
  typedef EdgeP                                              edge_property;
  typedef size_t                                             vertex_descriptor;
  typedef size_t                                      simple_vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor>            edge_descriptor;
  typedef typename sequential::select_edge
          <vertex_descriptor, edge_property,D>::type         edge_type;
  typedef adjacency_list_impl<edge_type>                     edgelist_type;
  typedef hgraph_vertex_adj_list_impl
          <vertex_descriptor,vertex_property,edgelist_type>  vertex_impl_type;
  typedef adjacency_list_hashmap_storage
          <hgraph_seq_graph_traits>                          storage_type;
  typedef typename sequential::graph_type
          <hgraph_seq_graph_traits,D>::type                  directness_type;
  typedef typename sequential::graph_type
          <hgraph_seq_graph_traits,M>::type                  multiplicity_type;
  static const graph_attributes d_type = D; //directedness
  static const graph_attributes m_type = M; //edge multiplicity
};


//////////////////////////////////////////////////////////////////////
/// @brief Traits class for @ref hgraph_base_container.
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
struct hgraph_base_container_traits
{
  typedef hgraph_base_container_traits<D,M,VertexP,EdgeP>    this_type;
  typedef VertexP                                        raw_vertex_property;
  typedef VertexP                                            vertex_property;
  typedef typename define_value_type<VertexP>::type          stored_type;
  typedef EdgeP                                              edge_property;
  typedef size_t                                             vertex_descriptor;
  typedef size_t                                      simple_vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor>            edge_descriptor;
  typedef typename sequential::select_edge
          <vertex_descriptor, edge_property,D>::type         edge_type;

  typedef hgraph_seq_graph_traits<D,M,VertexP,EdgeP>         seq_graph_traits_t;

  typedef typename seq_graph_traits_t::edgelist_type         edgelist_type;
  typedef typename seq_graph_traits_t::vertex_impl_type      vertex_impl_type;
  typedef adjacency_list_model<seq_graph_traits_t>           container_type;
  typedef typename sequential::graph_type<this_type,D>::type directness_type;
  typedef typename sequential::graph_type<this_type,M>::type multiplicity_type;
  static const graph_attributes d_type = D; //directedness
  static const graph_attributes m_type = M; //edge multiplicity

  typedef domset1D<vertex_descriptor>                        domain_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Base-container for a hierarchical graph. Inherits all functionality
/// from @ref graph_base_container.
/// @ingroup pgraphBaseCont
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedgedness
/// (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
/// Must be default assignable, copyable and assignable.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D,
          graph_attributes M,
          typename VertexP,
          typename EdgeP>
struct hgraph_base_container
  : public graph_base_container<hgraph_base_container_traits<D,M,VertexP,EdgeP>>
{
  typedef hgraph_base_container_traits<D,M,VertexP,EdgeP> traits_type;

  typedef graph_base_container<traits_type>                base_type;

  typedef typename traits_type::container_type             container_type;
  typedef VertexP                                          vertex_property;
  typedef typename traits_type::vertex_descriptor          vertex_descriptor;
  typedef typename traits_type::domain_type                domain_type;
  typedef size_t                                           cid_type;

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph base container with a given size and domain,
  /// and constructs all elements with a default value for vertex.
  /// @param domain Provides the domain of vertex descriptors stored in
  /// this base container.
  /// @param cid Provides the id of this base-container.
  /// @param default_value provides the default value for the vertices
  /// stored in this base-container.
  //////////////////////////////////////////////////////////////////////
  template<typename ConstructDomainType>
  hgraph_base_container(const ConstructDomainType& domain,
                        const cid_type& cid,
                        const typename base_type::value_type& default_value)
    : base_type(domain, cid, default_value)
  { }

  hgraph_base_container(const hgraph_base_container& other)
    : base_type(other)
  { }

  ~hgraph_base_container()
  {
    this->data()->clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex to the component with the given vertex property.
  /// The descriptor is assigned automatically.
  /// @param v The vertex property of the added vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(const vertex_property& v)
  {
    vertex_descriptor vid = this->data()->add_vertex(v);
    this->m_domain += vid;
    return vid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex to the component with the given descriptor and
  /// property.
  /// @param gid The explicit descriptor of the added vertex.
  /// @param v The vertex property of the added vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(const vertex_descriptor& gid,
                               const vertex_property& v)
  {
    this->m_domain += gid;
    return this->data()->add_vertex(gid, v);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the vertex with the specified gid.
  /// @param gid The descriptor of the vertex to be deleted.
  /// @return Whether or not the vertex was successfully deleted.
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(vertex_descriptor gid)
  {
    this->m_domain -= gid;
    return this->data()->delete_vertex(gid);
  }

}; //end hgraph_base_container

} //namespace STAPL

#endif /* STAPL_CONTAINERS_GRAPH_HGRAPH_BASE_CONTAINER_HPP */
