/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_BASE_CONTAINER_BASE_HPP
#define STAPL_CONTAINERS_GRAPH_BASE_CONTAINER_BASE_HPP

#include <stapl/containers/base/bc_base.hpp>
#include <stapl/domains/iterator_domain.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>
#include <stapl/containers/iterators/const_local_iterator.hpp>
#include <stapl/containers/graph/base_container/graph_storage.h>
#include <stapl/containers/graph/is_static.hpp>
#include <stapl/containers/graph/local_accessor_graph.hpp>
#include <stapl/containers/graph/const_local_accessor_graph.hpp>
#include <stapl/containers/sequential/graph/graph.h>
#include <stapl/containers/type_traits/define_value_type.hpp>
#include <boost/type_traits/is_same.hpp>

namespace stapl {

namespace graph_bc_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Called in destructor of @ref graph_base_container_base to destroy
///   heap value held by @ref container_wrapper_ref, if the value
///   type of the base container is a nested container.
//////////////////////////////////////////////////////////////////////
template<typename VertexProperty, bool = is_container<VertexProperty>::value>
struct cleanup_elements
{
  template<typename GraphStorage>
  static void apply(GraphStorage&, bool defer)
  { }
};


template<typename VertexProperty>
struct cleanup_elements<VertexProperty, true>
{
  template<typename GraphStorage>
  static void apply(GraphStorage& g, bool defer)
  {
    if (!defer)
    {
      typename GraphStorage::iterator it = g.begin();
      typename GraphStorage::iterator it_e = g.end();
      p_object_delete<VertexProperty> d;
      for ( ; it != it_e; ++it) {
        d(it->property().get().get_rmi_handle());
      }
    }
  }
};

} // namespace graph_bc_impl


template<graph_attributes D, graph_attributes M,
          typename VertexP, typename EdgeP, typename Traits>
struct graph_base_container_base;


//////////////////////////////////////////////////////////////////////
/// @todo Use inheritance of Traits idiom (see other base container)
///   to reduce code duplication (most of the types are already defined
///   there).
//////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M,
          typename VertexP, typename EdgeP, typename Traits>
struct container_traits<
  graph_base_container_base<D, M, VertexP, EdgeP, Traits>
>
{
  typedef typename Traits::vertex_impl_type              vertex_type;
  typedef typename Traits::vertex_descriptor             vertex_descriptor;
  typedef vertex_type                                    value_type;
  typedef typename Traits::domain_type                   domain_type;
  typedef vertex_descriptor                              gid_type;
  typedef typename Traits::container_type                container_type;
  typedef local_iterator<
    graph_base_container_base<D, M, VertexP, EdgeP, Traits>
  >                                                      iterator;
  typedef local_accessor_graph<
    graph_base_container_base<D, M, VertexP, EdgeP, Traits>
  >                                                      accessor_t;
  typedef proxy<value_type, accessor_t>                  reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief Traits class for static @ref graph_base_container.
/// @ingroup pgraphTraitsBaseContainer
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
/// Must be default assignable, copyable and assignable.
/// @tparam Dom Domain type of the base container.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M,
          typename VertexP, typename EdgeP, typename Dom>
struct graph_base_container_traits
{
private:
  typedef graph_base_container_traits<D,M,VertexP,EdgeP, Dom> this_type;

public:
  typedef VertexP                                     raw_vertex_property;
  typedef typename define_value_type<VertexP>::type           stored_type;
  typedef stored_type                                         vertex_property;
  typedef EdgeP                                               edge_property;
  typedef size_t                                              vertex_descriptor;
  typedef size_t                                       simple_vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor>             edge_descriptor;
  typedef typename sequential::select_edge<vertex_descriptor,
                               edge_property,D>::type         edge_type;
  typedef adjacency_list_impl<edge_type>                      edgelist_type;
  typedef vertex_adj_list_impl
          <vertex_descriptor,vertex_property,edgelist_type>   vertex_impl_type;

  typedef adjacency_list_vector_storage<this_type>            storage_type;
  typedef adjacency_list_model<this_type>                     container_type;
  typedef typename sequential::graph_type<this_type,D>::type  directness_type;
  typedef typename sequential::graph_type<this_type,M>::type  multiplicity_type;
  static const graph_attributes d_type = D; //directedness
  static const graph_attributes m_type = M; //edge multiplicity
  typedef Dom                                                 domain_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Base class for the graph's base-container.
/// Provides functionality of the graph API for local storage.
/// @tparam D graph-attribute specifying Directedness (DIRECTED/UNDIRECTED).
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge.
/// Must be default assignable, copyable and assignable.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the domain type, vertex type, edge type, edgelist type,
/// storage, etc. E.g. @ref graph_base_container_traits.
/// @ingroup pgraphBaseCont
///
/// @todo write edge_reference to access edges.
//////////////////////////////////////////////////////////////////////
template <graph_attributes D,
          graph_attributes M,
          typename VertexP,
          typename EdgeP,
          typename Traits>
struct graph_base_container_base
  : public bc_base
{
  typedef typename Traits::container_type             container_type;
  typedef typename Traits::vertex_property            vertex_property;
  typedef typename Traits::edge_property              edge_property;

  /// vertex_descriptor is the GID of the vertex.
  typedef typename Traits::vertex_descriptor          vertex_descriptor;

  /// @brief The simple vertex descriptor is an integral type.
  /// Convertible to/from vertex_descriptor.
  typedef typename Traits::simple_vertex_descriptor   simple_vertex_descriptor;
  typedef typename Traits::edge_descriptor            edge_descriptor;
  typedef typename Traits::directness_type            directness_type;
  typedef typename Traits::multiplicity_type          multiplicity_type;
  typedef typename Traits::domain_type                domain_type;

  /// For graphs, the global identifier is the vertex descriptor.
  typedef vertex_descriptor                           gid_type;
  typedef size_t                                      cid_type;

  /// The type of the vertex.
  typedef typename Traits::vertex_impl_type           vertex_type;
  /// The type of edgelist to use for each vertex.
  typedef typename Traits::edgelist_type              edgelist_type;
  /// The type of edge to use.
  typedef typename Traits::edge_type                  edge_type;

  /// For a graph, the vertex is the value-type.
  typedef vertex_type                                 value_type;

  typedef local_accessor_graph<
    graph_base_container_base
  >                                                   accessor_t;
  typedef proxy<vertex_type, accessor_t>              vertex_reference;
  typedef local_iterator<graph_base_container_base,
                         accessor_t>                  vertex_iterator;

  typedef vertex_reference                            reference;
  typedef vertex_iterator                             iterator;

  typedef const_local_accessor_graph<
    graph_base_container_base
  >                                                   const_accessor_t;
  typedef proxy<vertex_type, const_accessor_t>        const_vertex_reference;
  typedef const_vertex_reference                      const_reference;
  typedef const_local_iterator<graph_base_container_base,
                         const_accessor_t>            const_vertex_iterator;
  typedef const_vertex_iterator                       const_iterator;

  typedef typename edgelist_type::iterator            adj_edge_iterator;
  typedef typename edgelist_type::const_iterator      const_adj_edge_iterator;
  typedef sequential::edge_iterator_adaptor<vertex_iterator>    edge_iterator;

protected:
  /// The domain of this base_container
  domain_type            m_domain;

  //////////////////////////////////////////////////////////////////////
  /// @brief m_data is the model and storage for the graph. The model
  /// affects the performance and capabilities of the graph. For example,
  /// the @ref adjacency_list_model provides an adjacency list of vertices.
  //////////////////////////////////////////////////////////////////////
  container_type         m_data;

  /// This base_container's id
  cid_type               m_cid;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of domain initializer of this base-container.
  /// For static domains, this is a contiguous range of integers.
  /// @param domain Provides the initial domain of the base-container.
  //////////////////////////////////////////////////////////////////////
  template<typename Domain>
  void init_domain(Domain const& domain, boost::true_type)
  {
    if (domain.size() != 0)
      m_domain=domain_type(domain.first(), domain.last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specialization of domain initializer of this base-container.
  /// For iterator domains, the domain is over the data stored in this
  /// base-container.
  /// @param domain Provides the initial domain of the base-container.
  //////////////////////////////////////////////////////////////////////
  template<typename Domain>
  void init_domain(Domain const&, boost::false_type)
  {
    m_domain = domain_type(m_data);
  }

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
  graph_base_container_base(Domain const& domain, cid_type const& cid)
    : m_data(domain.first(), domain.size(), value_type()), m_cid(cid)
  {
    init_domain(domain,is_static<domain_type>());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph base container with a given size and domain
  /// and constructs all elements with a default value for vertex.
  /// @param domain Provides the domain of vertex descriptors stored in
  /// this base container.
  /// @param cid Provides the id of this base-container.
  /// @param default_value provides the default value for the vertices
  /// stored in this base-container.
  //////////////////////////////////////////////////////////////////////
  template<typename Domain>
  graph_base_container_base(Domain const& domain, cid_type const& cid,
                            value_type const& default_value)
    : m_data(domain.first(), domain.size(), default_value), m_cid(cid)
  {
    init_domain(domain,is_static<domain_type>());
  }

  ~graph_base_container_base(void)
  {
    // We use raw_vertex_property type here as this is the actual
    // type of the property intended by the user,
    // traits::vertex_property type is what is physically stored
    // in the base-containers and may be a wrapper around the
    // actual property in case the property is a pContainer.
    graph_bc_impl::cleanup_elements<
      typename Traits::raw_vertex_property>::apply(m_data,
                                                   this->m_defer_cleanup);
  }

  container_type* data()
  {
    return &m_data;
  }

  vertex_descriptor next_free_descriptor()
  {
    return m_data.next_free_descriptor();
  }

  vertex_iterator begin(void)
  {
    return vertex_iterator(this->m_data.begin(), this);
  }

  vertex_iterator end(void)
  {
    return vertex_iterator(this->m_data.end(), this);
  }

  edge_iterator edges_begin(void)
  {
    return edge_iterator(this->m_data.edges_begin(), this);
  }

  edge_iterator edges_end(void)
  {
    return edge_iterator(this->m_data.edges_end(), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex to the component with the given vertex property.
  /// The descriptor is assigned automatically.
  /// @param v The vertex property of the added vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_property const& v)
  {
    vertex_descriptor vid = this->m_data.add_vertex(v);
    this->m_domain += vid;
    return vid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates the vertex descriptor generator with the next free
  ///        descriptor
  //////////////////////////////////////////////////////////////////////
  void update_next_descriptor(vertex_descriptor const& vd)
  {
    this->m_data.update_next_descriptor(vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex to the component with the given descriptor and
  /// property.
  /// @param gid The explicit descriptor of the added vertex.
  /// @param v The vertex property of the added vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_descriptor const& gid,
                               vertex_property const& v)
  {
    vertex_descriptor vid = this->m_data.add_vertex(gid, v);
    this->m_domain += vid;
    return vid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex to the component with the given descriptor, property
  /// and edges.
  /// @param v The vertex to be added, along with edges and properties.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_element(vertex_type const& v)
  {
    vertex_descriptor vd =
      add_vertex_helper(v,
        typename boost::is_same<vertex_property, properties::no_property>()
      );

    typename vertex_type::const_adj_edge_iterator it = v.begin();
    typename vertex_type::const_adj_edge_iterator eit = v.end();

    for (; it != eit; ++it)
      add_edge_helper(*it,
        typename boost::is_same<edge_property, properties::no_property>()
      );


    return vd;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reserves space for adjacent edges of the vertex with the specified
  /// descriptor.
  /// @param gid The descriptor of the vertex which needs to reserve space.
  /// @param num_adjacents The number of adjacent edges for which space will
  /// be reserved.
  //////////////////////////////////////////////////////////////////////
  void reserve_adjacency(vertex_descriptor const& gid, size_t num_adjacents)
  {
    this->m_data.reserve_adjacency(gid, num_adjacents);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the vertex with the specified descriptor.
  /// @param gid The descriptor of the vertex to be deleted.
  /// @return Whether or not the vertex was successfully deleted.
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(vertex_descriptor gid)
  {
    this->m_domain -= gid;
    return this->m_data.delete_vertex(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the vertex associated with the specified descriptor,
  /// but leave all the edges pointing to it from other vertices. Needed
  /// for migration.
  /// @param gid identifier of the vertex to be suspended.
  /// @return Whether or not the vertex was successfully deleted.
  //////////////////////////////////////////////////////////////////////
  bool suspend_vertex(vertex_descriptor gid)
  {
    this->m_domain -= gid;
    return this->m_data.suspend_vertex(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Checks if the vertex with the specified descriptor is local to the
  /// component, then returns an iterator pointing to it.
  /// @param gid Descriptor of the vertex to be searched.
  /// @note This method should be called after we check if the element
  /// is local or not.
  /// @return The iterator pointing to the vertex, if found, else, an iterator
  /// to the end of the container is returned.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator find_vertex(vertex_descriptor const& gid)
  {
    return vertex_iterator(this->m_data.find_vertex(gid), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_base_container_base::find_vertex(vertex_descriptor const& gid)
  //////////////////////////////////////////////////////////////////////
  const_vertex_iterator find_vertex(vertex_descriptor const& gid) const
  {
    return const_vertex_iterator(this->m_data.find_vertex(gid), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Same as @ref find_vertex(vertex_descriptor const&).
  /// Provided for compatibility with other base containers.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator find(vertex_descriptor const& gid)
  {
    return vertex_iterator(this->m_data.find_vertex(gid), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge to this base-container with the given edge descriptor
  /// and edge property.
  /// @param edd The descriptor of edge to be added.
  /// @param ep The property of the edge to be added.
  /// @return The descriptor of the added edge. The id of the descriptor
  /// is invalid if the edge was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& edd, edge_property const& ep)
  {
    return this->m_data.add_edge(edd, ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge to this base-container from given source to target
  /// and edge property.
  /// @param source The vertex descriptor of source.
  /// @param target The vertex descriptor of target.
  /// @param ep The property of the edge to be added.
  /// @return The descriptor of the added edge. The id of the descriptor
  /// is invalid if the edge was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(vertex_descriptor const& source,
                           vertex_descriptor const& target,
                           edge_property const& ep)
  {
    return this->m_data.add_edge(edge_descriptor(source,target), ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts an edge into this base-container with the given edge
  /// descriptor and edge property, using a comparator function to determine
  /// where in edge-container of a vertex the edge will be inserted. (Requires
  /// the edges of the source vertex to already be sorted by the comparator.)
  /// @param edd The descriptor of edge to be inserted.
  /// @param ep The property of the edge to be inserted.
  /// @param comp Passed internal to std::lower_bound to find the spot to insert
  /// the new edge.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  edge_descriptor insert_edge(edge_descriptor const& edd,
                              edge_property const& ep, Comp const& comp)
  {
    return this->m_data.insert_edge(edd, ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts an edge into this base-container from given source to
  /// target descriptor and edge property, using a comparator function to
  /// determine where in edge-container of a vertex the edge will be inserted.
  /// (Requires the edges of the source vertex to already be sorted by the
  /// comparator.)
  /// @param source The vertex descriptor of source.
  /// @param target The vertex descriptor of target.
  /// @param ep The property of the edge to be inserted.
  /// @param comp Passed internal to std::lower_bound to find the spot to insert
  /// the new edge.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  edge_descriptor insert_edge(vertex_descriptor const& source,
                              vertex_descriptor const& target,
                              edge_property const& ep, Comp const& comp)
  {
    return this->m_data.insert_edge(edge_descriptor(source,target), ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_base_container_base::insert_edge(edge_descriptor const &, edge_property const&, Comp const&)
  /// @note Used for compatibility
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_async(edge_descriptor const& edd, edge_property const& ep,
                         Comp const& comp)
  {
    this->m_data.insert_edge(edd, ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_base_container_base::insert_edge(vertex_descriptor const&, vertex_descriptor const&, edge_property const&, Comp const&)
  /// @note Used for API compatibility
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_async(vertex_descriptor const& source,
                         vertex_descriptor const& target,
                         edge_property const& ep, Comp const& comp)
  {
    this->m_data.insert_edge(edge_descriptor(source,target), ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_base_container_base::add_edge(edge_descriptor const &, edge_property const&, Comp const&)
  /// @note Used for compatibility
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge_async(edge_descriptor const& edd,
                                 edge_property const& ep)
  {
    return this->m_data.add_edge(edd, ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_base_container_base::add_edge(vertex_descriptor const&, vertex_descriptor const&, edge_property const&, Comp const&)
  /// @note Used for API compatibility
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge_async(vertex_descriptor const& source,
                                 vertex_descriptor const& target,
                                 edge_property const& ep)
  {
    return this->m_data.add_edge(edge_descriptor(source,target), ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge to this base-container with the given edge descriptor
  /// and edge property. If bidir is true, also adds the sibling edge, if the
  /// target also exists in this base-container.
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
      return this->m_data.add_edge(edd, ep);
    else
    {
      if (edd.id() == INVALID_VALUE)
        return this->m_data.add_edge(edd, ep);
      else
      {
        add_internal_edge(this->m_data, edd, ep);
        return edd;
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes an edge from this base-container with the given descriptor.
  /// @return True if the edge was deleted, false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool delete_edge(edge_descriptor const& ed)
  {
    return this->m_data.delete_edge(ed);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the edge with the given edge descriptor.
  /// @param ed The descriptor of the edge to be found.
  /// @param lit The vertex iterator that will be updated to point to
  /// the source-vertex of the edge, if it was found.
  /// @param ei The iterator that will be updated to point to the edge,
  /// if found.
  /// @return True if the edge was found, false otherwise.
  /// If found, lit and ei will be valid and pointing to the corresponding
  /// source vertex and the edge.
  //////////////////////////////////////////////////////////////////////
  bool find_edge(edge_descriptor const& ed, vertex_iterator lit,
                 adj_edge_iterator ei)
  {
    return this->m_data.find_edge(ed, lit, ei);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Determines if an edge exists between two vertices. This method
  ///        is similar to find_edge, but does not assign iterators.
  //////////////////////////////////////////////////////////////////////
  bool has_edge(vertex_descriptor const& source,
                vertex_descriptor const& target)
  {
    edge_descriptor ed{source, target};
    typename container_type::vertex_iterator it;
    typename container_type::adj_edge_iterator adj_it;

    return this->m_data.find_edge(ed, it, adj_it);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief The number of vertices in this base-container.
  /// @return size_t The number of vertices in this base-container.
  //////////////////////////////////////////////////////////////////////
  size_t num_vertices(void) const
  {
    return this->m_data.num_vertices();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief The number of vertices in this base-container.
  /// Same as @ref num_vertices(), provided for compatibility.
  /// @return size_t The number of vertices in this base-container.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->m_data.num_vertices();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Tests if the base-container is empty or not.
  /// @return true if the component is empty and false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return this->m_data.num_vertices() == 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes all vertices and edges in this base-container.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->m_data.clear();
    this->m_domain = domain_type();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sorts edges of each vertex based on given comparator.
  /// @param comp The comparator used to sort edges.
  //////////////////////////////////////////////////////////////////////
  template<typename Comp>
  void sort_edges(Comp const& comp)
  {
    this->m_data.sort_edges(comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases all edges that match a user-defined predicate
  /// @param pred A unary predicate that receives a single edge
  //////////////////////////////////////////////////////////////////////
  template<typename Pred>
  void erase_edges_if(Pred&& pred)
  {
    this->m_data.erase_edges_if(std::forward<Pred>(pred));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all duplicate edges with the same (source, target) pair
  //////////////////////////////////////////////////////////////////////
  void remove_duplicate_edges()
  {
    this->m_data.remove_duplicate_edges();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief The number of edges in this base-container.
  //////////////////////////////////////////////////////////////////////
  size_t num_edges(void) const
  {
    return this->m_data.num_edges();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of self edges in the base-container managed.
  //////////////////////////////////////////////////////////////////////
  size_t num_self_edges(void) const
  {
    return this->m_data.num_self_edges();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a bidirectional edge to the base-container with the given
  /// descriptor and property. The sibling edge has the source and target
  /// reversed. Both vertices should be in this base-container.
  /// @param ed The descriptor of the edge to be added. If the id is not
  /// invalid, it will be used to create the edges, otherwise one will be
  /// generated.
  /// @param p The property of the edge to be added.
  /// @return The descriptor of edge added. The source is the smaller of
  /// the source and target descriptors.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_bidirectional_edge(edge_descriptor const& ed,
                                         edge_property const& p)
  {
    return add_edge_pair(this->m_data, ed, p);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief applies a functor f on the specified vertex.
  /// @param gid the descriptor of the vertex on which the functor will be
  /// applied.
  /// @param f the functor to be applied.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  void apply(vertex_descriptor const& gid, F const& f)
  {
    f(this->make_reference(gid));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief applies a functor f on the specified vertex.
  /// @param gid the descriptor of the vertex on which the functor will be
  /// applied.
  /// @param f the functor to be applied.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  void apply_set(vertex_descriptor const& gid, F const& f)
  {
    f(this->make_reference(gid));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief applies a functor f on the specified vertex and returns a value.
  /// @param gid the descriptor of the vertex on which the functor will be
  /// applied.
  /// @param f the functor to be applied.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  typename F::result_type apply_get(vertex_descriptor const& gid, F const& f)
  {
    return f.template operator()<value_type&>(*(m_data.find_vertex(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief applies a functor f on the specified vertex and returns a value.
  /// @param gid the descriptor of the vertex on which the functor will be
  /// applied.
  /// @param f the functor to be applied.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  typename F::result_type
  apply_get(vertex_descriptor const& gid, F const& f) const
  {
    return f.template operator()<value_type const&>(*(m_data.find_vertex(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a functor f on the property of the specified vertex.
  /// @param gid The descriptor of the vertex on which the functor will be
  /// applied.
  /// @param f The functor to be applied.
  /// @return The result of applying f on the vertex property.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  typename F::result_type vp_apply(vertex_descriptor const& gid, F const& f)
  {
    return f((*(m_data.find_vertex(gid))).property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a functor f on the property of the specified vertex.
  /// @param gid The descriptor of the vertex on which the functor will be
  /// applied.
  /// @param f The functor to be applied.
  /// @return The result of applying f on the vertex property.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  typename F::result_type
  vp_apply(vertex_descriptor const& gid, F const& f) const
  {
    return f((*(m_data.find_vertex(gid))).property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the property of the specified vertex to the given value.
  /// @param gid The descriptor of the vertex.
  /// @param vp The new value of the property.
  //////////////////////////////////////////////////////////////////////
  void vp_set(vertex_descriptor const& gid,
              typename Traits::stored_type const& vp)
  {
    (*(m_data.find_vertex(gid))).property() = vp;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a functor f on the property of the specified vertex.
  /// @param gid The descriptor of the vertex on which the functor will be
  /// applied.
  /// @param f The functor to be applied.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  void vp_apply_async(vertex_descriptor const& gid, F const& f)
  {
    f((*(m_data.find_vertex(gid))).property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies each element from the given container of functors on the
  /// property of the vertex specified by each element.
  /// @param fcont The container of functors to be applied. Elements must
  /// additionally provide a target() method to specify the descriptor
  /// of the vertex on which the functor will be applied.
  //////////////////////////////////////////////////////////////////////
  template <typename FCont>
  void aggregate_vp_apply(FCont const& fcont)
  {
    for (typename FCont::const_iterator it = fcont.begin(); it != fcont.end();
         ++it)
      it->operator() (m_data.find_vertex(it->target())->property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a functor f on the property of the specified edge, if it
  /// exists.
  /// @param ed The descriptor of the edge on which the functor will be
  /// applied.
  /// @param f The functor to be applied.
  /// @return True if the edge exists, False otherwise.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  bool ep_find_apply(edge_descriptor const& ed, F const& f)
  {
    typename Traits::container_type::vertex_iterator it;
    adj_edge_iterator eit;
    if (m_data.find_edge(ed, it, eit)) {
      f((*eit).property());
      return true;
    } else {
      return false;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a functor f on the property of the specified edge.
  /// @param ed The descriptor of the edge on which the functor will be
  /// applied.
  /// @param f The functor to be applied.
  /// @note The edge must exist.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  void ep_apply(edge_descriptor const& ed, F const& f)
  {
    typename Traits::container_type::vertex_iterator it;
    adj_edge_iterator eit;
    if (m_data.find_edge(ed, it, eit))
      f((*eit).property());
    else
      stapl_assert(false, "accessing an edge that does not exist.");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a functor f on the property of the specified edge, and
  /// returns the result.
  /// @param ed The descriptor of the edge on which the functor will be
  /// applied.
  /// @param f The functor to be applied.
  /// @return The result of the application of f to the specified edge.
  /// @note The edge must exist.
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  typename F::result_type ep_apply_get(edge_descriptor const& ed, F const& f)
  {
    typename Traits::container_type::vertex_iterator it;
    adj_edge_iterator eit;
    if (m_data.find_edge(ed, it, eit))
      return f((*eit).property());
    else {
      stapl_assert(false, "accessing an edge that does not exist.");
      return typename F::result_type();
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the specified vertex.
  /// @param g The descriptor of the vertex.
  /// @return The reference to the specified vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_reference operator[](gid_type const& g)
  {
    return make_reference(g);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the specified vertex.
  /// @param g The descriptor of the vertex.
  /// @return The reference to the specified vertex.
  //////////////////////////////////////////////////////////////////////
  const_vertex_reference operator[](gid_type const& g) const
  {
    return make_reference(g);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the specified vertex.
  /// @param g The descriptor of the vertex.
  /// @return The reference to the specified vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_reference make_reference(vertex_descriptor const& gid)
  {
    return vertex_reference(accessor_t(this, m_data.find_vertex(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the specified vertex.
  /// @param g The descriptor of the vertex.
  /// @return The reference to the specified vertex.
  //////////////////////////////////////////////////////////////////////
  const_vertex_reference make_reference(vertex_descriptor const& gid) const
  {
    return const_vertex_reference(
             const_accessor_t(this, m_data.find_vertex(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified vertex.
  /// @param gid The descriptor of the vertex.
  /// @return The iterator to the specified vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator make_iterator(vertex_descriptor const& gid)
  {
    return vertex_iterator(m_data.find_vertex(gid), this);
  }

  container_type& container(void)
  {
    return m_data;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the id of this base-container.
  //////////////////////////////////////////////////////////////////////
  cid_type get_cid(void) const
  {
    return m_cid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the id of this base-container.
  /// Same as get_cid for compatibility.
  //////////////////////////////////////////////////////////////////////
  cid_type cid(void) const
  {
    return m_cid;
  }

  void set_cid(cid_type cid)
  {
    m_cid = cid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Define type for graph_base_container
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.base<bc_base>(*this);
    t.member(m_data);
    t.member(m_cid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the domain of this base-container.
  //////////////////////////////////////////////////////////////////////
  domain_type const& domain(void) const
  {
    return m_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the position of the given vertex in this base-container.
  //////////////////////////////////////////////////////////////////////
  gid_type local_position(gid_type const& gid) const
  {
    stapl_assert(this->domain().contains(gid),
                 "attempting to access an element that is out of"
                 "bounds of the base container");
    return gid-this->domain().first();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the specified vertex.
  /// @param gid The descriptor of the vertex.
  /// @return The reference to the specified vertex.
  //////////////////////////////////////////////////////////////////////
  value_type get_element(gid_type const& gid)
  {
    return make_reference(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a const reference to the specified vertex.
  /// @param gid The descriptor of the vertex.
  /// @return The reference to the specified vertex.
  //////////////////////////////////////////////////////////////////////
  const_reference get_element(gid_type const& gid) const
  {
    return make_reference(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the element corresponding to a specific GID.
  ///
  /// This method is used in the redistribution of composed containers.
  /// It is needed to allow the distributor object to get the instance of
  /// the @ref container_wrapper_ref for a container instance on one location
  /// and send it to another location where it will be placed in a base
  /// container by calling set_element. @ref set_element only accepts
  /// instances of the stored type.
  ///
  /// @param gid The id associated with the element for which we want to read
  /// the value.
  ///
  /// @return A reference to the stored instance of the specified vertex.
  //////////////////////////////////////////////////////////////////////
  reference get_stored_element(gid_type const& gid)
  {
    return make_reference(gid);
  }

private:

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a vertex to this base-container. Specialized for when
  /// the vertex does not have a property.
  /// @param v The vertex object to be added.
  /// @return The descriptor of the new vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex_helper(vertex_type const& v, boost::true_type)
  {
    this->m_domain += v.descriptor();
    return this->m_data.add_vertex(v.descriptor(), vertex_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a vertex to this base-container. Specialized for when
  /// the vertex has a property.
  /// @param v The vertex object to be added.
  /// @return The descriptor of the new vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex_helper(vertex_type const& v, boost::false_type)
  {
    this->m_domain += v.descriptor();
    return this->m_data.add_vertex(v.descriptor(), v.property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge to this base-container. Specialized for when
  /// the edge does not have a property.
  /// @param e The edge object to be added.
  //////////////////////////////////////////////////////////////////////
  void add_edge_helper(edge_type const& e, boost::true_type)
  {
    this->add_edge(e.descriptor(), edge_property(), false);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge to this base-container. Specialized for when
  /// the edge has a property.
  /// @param e The edge object to be added.
  //////////////////////////////////////////////////////////////////////
  void add_edge_helper(edge_type const& e, boost::false_type)
  {
    this->add_edge(e.descriptor(), e.property(), false);
  }

}; // class graph_base_container_base

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_BASE_CONTAINER_BASE_HPP
