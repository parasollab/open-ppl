/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_GRAPH_BASE_CONTAINER_HPP


#include <stapl/containers/graph/base_container/base_container_base.hpp>


namespace stapl {


//////////////////////////////////////////////////////////////////////
/// @brief Class for Non-Multiedged graph's base-container.
/// Derives from the @ref graph_base_container_base class and overloads add_edge
/// method for checking multiple edges.
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam VertexP type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
/// Must be default assignable, copyable and assignable.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, vertex type, edge type, edgelist type,
///  storage, etc. E.g. @ref graph_base_container_traits.
/// @ingroup pgraphBaseCont
//////////////////////////////////////////////////////////////////////
template <graph_attributes D,
          typename VertexP,
          typename EdgeP,
          typename Traits>
struct graph_base_container_NME
  : public graph_base_container_base<D, stapl::NONMULTIEDGES, VertexP, EdgeP,
                                     Traits>
{
  typedef typename Traits::domain_type                domain_type;
  typedef graph_base_container_base<
            D, stapl::NONMULTIEDGES, VertexP, EdgeP,
            Traits>                                   base_type;
  typedef typename base_type::value_type              value_type;
  typedef typename base_type::edge_descriptor         edge_descriptor;
  typedef typename base_type::edge_property           edge_property;
  typedef size_t                                      cid_type;

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph base container with a given size and domain.
  ///
  /// This constructor is used during redistribution, where a default value
  /// for the vertex property isn't available.
  ///
  /// @param domain Provides the domain of vertex-descriptors stored in
  /// this base container.
  /// @param cid Provides the id of this base-container.
  //////////////////////////////////////////////////////////////////////
  template<typename Domain>
  graph_base_container_NME(Domain const& domain, cid_type const& cid)
    : base_type(domain, cid)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph base container with a given size and domain
  /// and constructs all elements with a default value for vertex.
  /// @param domain Provides the domain of vertex-descriptors stored in
  /// this base container.
  /// @param cid Provides the id of this base-container.
  /// @param default_value provides the default value for the vertices
  /// stored in this base-container.
  //////////////////////////////////////////////////////////////////////
  template<typename Domain>
  graph_base_container_NME(Domain const& domain, cid_type const& cid,
                           value_type const& default_value)
    : base_type(domain, cid, default_value)
  { }

  graph_base_container_NME(graph_base_container_NME const& other)
    : base_type(other)
  { }

  graph_base_container_NME() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge to this base-container with the given edge descriptor
  /// and edge property only if an edge with the same descriptor does not
  /// exist.
  /// @param edd The descriptor of edge to be added.
  /// @param ep The property of the edge to be added.
  /// @return The descriptor of the added edge. The id of the descriptor
  /// is invalid if the edge was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& edd, edge_property const& ep)
  {
    return this->data()->check_add_edge(edd, ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge to this base-container with the given edge descriptor
  /// and edge property. If bidir is true, also adds the sibling edge, if the
  /// target also exists in this base-container. The edges are added only if
  /// an edge with the same descriptor does not exist.
  /// @param edd The descriptor of edge to be added.
  /// @param ep The property of the edge to be added.
  /// @param bidir Whether or not to add the undirected sibling edge.
  /// @return The descriptor of the added edge. The id of the descriptor
  /// is invalid if the edge was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& edd, edge_property const& ep,
                           bool bidir)
  {
    if (!bidir)
      return this->data()->check_add_edge(edd, ep);
    else {
      if (edd.id() == INVALID_VALUE)
        return this->data()->check_add_edge(edd, ep);
      else {
        add_internal_edge(*(this->data()), edd, ep);
        return edd;
      }
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class for selecting between multi-edged and non-multi-edged graphs.
/// @tparam D graph-attribute specifying Multiedge. (DIRECTED/UNDIRECTED).
/// @tparam VertexP type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
/// Must be default assignable, copyable and assignable.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, vertex type, edge type, edgelist type,
///  storage, etc. E.g. @ref graph_base_container_traits.
/// @tparam MultiType graph-attribute specifying Multi-edgedness
/// (MULTIEDGES/NONMULTIEDGES).
/// @ingroup pgraphImpl
/// @todo This could implement one of the specializations (probably MULTIEDGES),
/// and we can get rid of one of the specializations.
//////////////////////////////////////////////////////////////////////
template<typename Traits,
         graph_attributes MultiType>
struct graph_multiplicity_base_container_selector
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref graph_multiplicity_base_container_selector for
/// NONMULTIEDGES graph.
/// @tparam D graph-attribute specifying Multiedge. (DIRECTED/UNDIRECTED).
/// @tparam VertexP type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
/// Must be default assignable, copyable and assignable.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, vertex type, edge type, edgelist type,
///  storage, etc. E.g. @ref graph_base_container_traits.
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template<typename Traits>
struct graph_multiplicity_base_container_selector<Traits,
                                                  NONMULTIEDGES>
{
  typedef graph_base_container_NME<Traits::d_type,
                                   typename Traits::vertex_property,
                                   typename Traits::edge_property,
                                   Traits> type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref graph_multiplicity_base_container_selector for
/// MULTIEDGES graph.
/// @tparam D graph-attribute specifying Multiedge. (DIRECTED/UNDIRECTED).
/// @tparam VertexP type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
/// Must be default assignable, copyable and assignable.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, vertex type, edge type, edgelist type,
///  storage, etc. E.g. @ref graph_base_container_traits.
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template<typename Traits>
struct graph_multiplicity_base_container_selector<Traits,
                                                  MULTIEDGES>
{
  typedef graph_base_container_base<Traits::d_type,
                                    MULTIEDGES,
                                    typename Traits::vertex_property,
                                    typename Traits::edge_property,
                                    Traits> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Graph's base-container. Inherits all functionality from either
/// @ref graph_base_container_base or @ref graph_base_container_NME class.
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedgedness
/// (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
/// Must be default assignable, copyable and assignable.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, vertex type, edge type, edgelist type,
///  storage, etc. E.g. @ref graph_base_container_traits.
/// @ingroup pgraphBaseCont
//////////////////////////////////////////////////////////////////////
template <typename Traits>
struct graph_base_container
  : public graph_multiplicity_base_container_selector<
      Traits, Traits::m_type>::type
{
  typedef graph_base_container<Traits>                          this_type;
  typedef typename Traits::domain_type                          domain_type;
  typedef size_t                                                cid_type;

  typedef typename graph_multiplicity_base_container_selector<
    Traits, Traits::m_type>::type                               base_type;
  typedef typename base_type::value_type                        value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph base container with a given size and domain.
  ///
  /// This constructor is used during redistribution, where a default value
  /// for the vertex property isn't available.
  ///
  /// @param domain Provides the domain of vertex-descriptors stored in
  /// this base container.
  /// @param cid Provides the id of this base-container.
  //////////////////////////////////////////////////////////////////////
  template <typename Domain>
  graph_base_container(Domain const& domain, cid_type const& cid)
    : base_type(domain, cid)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph base container with a given size and domain
  /// and constructs all elements with a default value for vertex.
  /// @param domain Provides the domain of vertex-descriptors stored in
  /// this base container.
  /// @param cid Provides the id of this base-container.
  /// @param default_value provides the default value for the vertices
  /// stored in this base-container.
  //////////////////////////////////////////////////////////////////////
  template <typename Domain>
  graph_base_container(Domain const& domain, cid_type const& cid,
                       value_type const& default_value)
    : base_type(domain, cid, default_value) { }

  graph_base_container(graph_base_container const& other)
    : base_type(other)
  { }

  graph_base_container() = default;

}; // struct graph_base_container

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_BASE_CONTAINER_HPP
