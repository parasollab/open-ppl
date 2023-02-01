/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_BASE_CONTAINER_STORAGE_HPP
#define STAPL_CONTAINERS_GRAPH_BASE_CONTAINER_STORAGE_HPP

#include <stapl/containers/graph/base_container/base_container_base.hpp>
#include <stapl/containers/graph/functional.hpp>

namespace stapl {

constexpr std::size_t commit_load(void) {
  return 4*1024;
}

constexpr std::size_t commit_buffer(void) {
  return 100000;
}

template<graph_attributes D, graph_attributes M,
          typename VertexP, typename EdgeP, typename Traits>
struct graph_base_container_storage_base;


template<graph_attributes D, graph_attributes M,
          typename VertexP, typename EdgeP, typename Traits>
struct container_traits<
  graph_base_container_storage_base<D, M, VertexP, EdgeP, Traits>
>
{
  typedef typename Traits::vertex_impl_type              vertex_type;
  typedef typename Traits::vertex_descriptor             vertex_descriptor;
  typedef vertex_type                                    value_type;
  typedef typename Traits::domain_type                   domain_type;
  typedef vertex_descriptor                              gid_type;
  typedef typename Traits::container_type                container_type;
  typedef local_iterator<
    graph_base_container_storage_base<D, M, VertexP, EdgeP, Traits>
  >                                                      iterator;
  typedef local_accessor_graph<
    graph_base_container_storage_base<D, M, VertexP, EdgeP, Traits>
  >                                                      accessor_t;
  typedef proxy<value_type, accessor_t>                  reference;
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
//////////////////////////////////////////////////////////////////////
template <graph_attributes D,
          graph_attributes M,
          typename VertexP,
          typename EdgeP,
          typename Traits>
struct graph_base_container_storage_base
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
    graph_base_container_storage_base
  >                                                   accessor_t;
  typedef proxy<vertex_type, accessor_t>              vertex_reference;
  typedef local_iterator<graph_base_container_storage_base,
                         accessor_t>                  vertex_iterator;

  typedef vertex_reference                            reference;
  typedef vertex_iterator                             iterator;

  typedef const_local_accessor_graph<
    graph_base_container_storage_base
  >                                                   const_accessor_t;
  typedef proxy<vertex_type, const_accessor_t>        const_vertex_reference;
  typedef const_vertex_reference                      const_reference;
  typedef const_local_iterator<graph_base_container_storage_base,
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
  container_type*         m_data;
  bool                    m_initialized;

  /// This base_container's id
  cid_type                m_cid;

  /// For buffering stored commits to the graph.
  char                    m_commit_buffer[commit_buffer()];
  size_t                  m_num_buffered;

  /// For buffering stored edge-commits to the graph.
  char                    m_edges_buffer[commit_buffer()];
  size_t                  m_num_buffered_edges;

  /// Write-back cache for hub-vertices.
  class cached_vertex_impl_type
  {
    vertex_property m_prop;

  public:
    vertex_property& property()
    { return m_prop; }

    vertex_property const& property() const
    { return m_prop; }

    size_t size(void) const
    { return 0; }
  };

  // A cache to store hub vertices in RAM when base-container is out-of-core.
  std::unordered_map<vertex_descriptor, cached_vertex_impl_type>  m_hubs_cache;

  /// Indicates whether this base-container is completely loaded in memory.
  bool                    m_loaded;
  /// Indicates whether or not this base-container's vertices have been loaded
  /// in memory.
  bool                    m_vertices_loaded;
  /// Indicates whether or not this base-container's edges have been loaded
  /// in memory.
  bool                    m_edge_loaded;
  /// Indicates whether any updates were performed for this base-container.
  bool                    m_active;
  /// Indicates whether any structure-modifications were done for this
  /// base-container.
  bool                    m_graph_structure_modifications;
  /// The name of the file to read/write this base-container from/to.
  char m_bc_fname[30];
  /// The name of the file to read/write this base-container's vertex-properties
  ///  from/to.
  char m_bc_vp_fname[30];

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
    m_domain = domain_type(*data());
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a graph base container with a given size and domain
  /// but does not construct elements.
  ///
  /// @ref initialize() must be called when loading for the first time
  /// to construct elements.
  /// @param domain Provides the domain of vertex descriptors stored in
  /// this base container.
  /// @param cid Provides the id of this base-container.
  /// @param default_value provides the default value for the vertices
  /// stored in this base-container.
  //////////////////////////////////////////////////////////////////////
  template<typename Domain>
  graph_base_container_storage_base(Domain const& domain, cid_type const& cid,
                            value_type const& default_value = value_type())
    : m_data(NULL),
      m_initialized(false),
      m_cid(cid)
  {
    init_domain(domain,is_static<domain_type>());
    init_loaded_state();
    init_fnames();
  }

  graph_base_container_storage_base(
    graph_base_container_storage_base const& other)
    : m_domain(other.m_domain),
      m_data(NULL),
      m_initialized(other.m_initialized),
      m_cid(other.m_cid)
  {
    if (m_initialized)
     m_data = new container_type(*other.m_data);
    init_loaded_state();
    init_fnames();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes variables dealing with loaded state.
  //////////////////////////////////////////////////////////////////////
  void init_loaded_state()
  {
    m_num_buffered = 0;
    m_num_buffered_edges = 0;
    m_loaded = false;
    m_vertices_loaded = false;
    m_edge_loaded = false;
    m_active = false;
    m_graph_structure_modifications = false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes variables indicating file-names related to this
  /// base-container.
  //////////////////////////////////////////////////////////////////////
  void init_fnames()
  {
    char buffer[12];
    sprintf(buffer,"%lu",m_cid);
    strcpy(m_bc_fname, "bc.");
    strcat(m_bc_fname, buffer);
    strcat(m_bc_fname, ".graph\0");

    strcpy(m_bc_vp_fname, "bc.");
    strcat(m_bc_vp_fname, buffer);
    strcat(m_bc_vp_fname, ".vp\0");
  }

  ~graph_base_container_storage_base(void)
  {
    // We use raw_vertex_property type here as this is the actual
    // type of the property intended by the user,
    // traits::vertex_property type is what is physically stored
    // in the base-containers and may be a wrapper around the
    // actual property in case the property is a pContainer.
    graph_bc_impl::cleanup_elements<
      typename Traits::raw_vertex_property>::apply(*data(),
                                                   this->m_defer_cleanup);

    delete m_data;
    m_data = NULL;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs all elements of the current base-container.
  //////////////////////////////////////////////////////////////////////
  bool initialize()
  {
    if (!m_initialized) {
      m_data = new container_type(m_domain.first(), m_domain.size());
      m_initialized = true;
      this->set_loaded(true);
      return true;
    }
    return false;
  }

  container_type* data()
  {
    return m_data;
  }

  vertex_descriptor next_free_descriptor()
  {
    return data()->next_free_descriptor();
  }

  vertex_iterator begin(void)
  {
    return vertex_iterator(this->data()->begin(), this);
  }

  vertex_iterator end(void)
  {
    return vertex_iterator(this->data()->end(), this);
  }

  edge_iterator edges_begin(void)
  {
    return edge_iterator(this->data()->edges_begin(), this);
  }

  edge_iterator edges_end(void)
  {
    return edge_iterator(this->data()->edges_end(), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds vertex to the component with the given vertex property.
  /// The descriptor is assigned automatically.
  /// @param v The vertex property of the added vertex.
  /// @return The descriptor of the added vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_property const& v)
  {
    return this->data()->add_vertex(v);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Updates the vertex descriptor generator with the next free
  ///        descriptor
  //////////////////////////////////////////////////////////////////////
  void update_next_descriptor(vertex_descriptor const& vd)
  {
    if (!this->is_vertices_loaded()) {
      return;
    }
    this->data()->update_next_descriptor(vd);
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
    return this->data()->add_vertex(gid, v);
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
    this->data()->reserve_adjacency(gid, num_adjacents);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the vertex with the specified descriptor.
  /// @param gid The descriptor of the vertex to be deleted.
  /// @return Whether or not the vertex was successfully deleted.
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(vertex_descriptor gid)
  {
   return this->data()->delete_vertex(gid);
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
   return this->data()->suspend_vertex(gid);
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
    return vertex_iterator(this->data()->find_vertex(gid), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_base_container_storage_base::find_vertex(vertex_descriptor const& gid)
  //////////////////////////////////////////////////////////////////////
  const_vertex_iterator find_vertex(vertex_descriptor const& gid) const
  {
    return const_vertex_iterator(this->data()->find_vertex(gid), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Same as @ref find_vertex(vertex_descriptor const&).
  /// Provided for compatibility with other base containers.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator find(vertex_descriptor const& gid)
  {
    return vertex_iterator(this->data()->find_vertex(gid), this);
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
    this->m_active = true;
    if (!this->is_fully_loaded()) {
      this->store_outstanding_edges(edd, ep);
      return edd;
    } else {
      this->m_graph_structure_modifications = true;
      return this->data()->add_edge(edd, ep);
    }
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
    return this->data()->add_edge(edge_descriptor(source,target), ep);
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
    return this->data()->insert_edge(edd, ep, comp);
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
    return this->data()->insert_edge(edge_descriptor(source,target), ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_base_container_storage_base::insert_edge(edge_descriptor const &, edge_property const&, Comp const&)
  /// @note Used for compatibility
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_async(edge_descriptor const& edd, edge_property const& ep,
                         Comp const& comp)
  {
    this->data()->insert_edge(edd, ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_base_container_storage_base::insert_edge(vertex_descriptor const&, vertex_descriptor const&, edge_property const&, Comp const&)
  /// @note Used for API compatibility
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_async(vertex_descriptor const& source,
                         vertex_descriptor const& target,
                         edge_property const& ep, Comp const& comp)
  {
    this->data()->insert_edge(edge_descriptor(source,target), ep, comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_base_container_storage_base::add_edge(edge_descriptor const &, edge_property const&, Comp const&)
  /// @note Used for compatibility
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge_async(edge_descriptor const& edd,
                                 edge_property const& ep)
  {
    return this->data()->add_edge(edd, ep);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_base_container_storage_base::add_edge(vertex_descriptor const&, vertex_descriptor const&, edge_property const&, Comp const&)
  /// @note Used for API compatibility
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge_async(vertex_descriptor const& source,
                                 vertex_descriptor const& target,
                                 edge_property const& ep)
  {
    return this->data()->add_edge(edge_descriptor(source,target), ep);
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
    this->m_active = true;
    if (!this->is_fully_loaded()) {
      this->store_outstanding_edges(edd, ep, bidir);
      return edd;
    } else {

      this->m_graph_structure_modifications = true;

      if (!bidir)
        return this->data()->add_edge(edd, ep);
      else {
        if (edd.id() == INVALID_VALUE)
          return this->data()->add_edge(edd, ep);
        else {
          add_internal_edge(*(this->data()), edd, ep);
          return edd;
        }
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes an edge from this base-container with the given descriptor.
  /// @return True if the edge was deleted, false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool delete_edge(edge_descriptor const& ed)
  {
    this->m_active = true;
    this->m_graph_structure_modifications = true;
    return this->data()->delete_edge(ed);
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
    return this->data()->find_edge(ed, lit, ei);
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

    return this->data()->find_edge(ed, it, adj_it);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief The number of vertices in this base-container.
  /// @return size_t The number of vertices in this base-container.
  //////////////////////////////////////////////////////////////////////
  size_t num_vertices(void) const
  {
    return this->data()->num_vertices();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief The number of vertices in this base-container.
  /// Same as @ref num_vertices(), provided for compatibility.
  /// @return size_t The number of vertices in this base-container.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    if (this->is_vertices_loaded())
     return this->data()->num_vertices();
    else return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Tests if the base-container is empty or not.
  /// @return true if the component is empty and false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    if (this->is_vertices_loaded())
      return this->data()->num_vertices() == 0;
    else return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes all vertices and edges in this base-container.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->m_active = true;
    this->m_graph_structure_modifications = true;
    if (this->is_vertices_loaded())
      this->data()->clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sorts edges of each vertex based on given comparator.
  /// @param comp The comparator used to sort edges.
  //////////////////////////////////////////////////////////////////////
  template<typename Comp>
  void sort_edges(Comp const& comp)
  {
    if (this->is_edges_loaded())
      this->data()->sort_edges(comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases all edges that match a user-defined predicate
  /// @param pred A unary predicate that receives a single edge
  //////////////////////////////////////////////////////////////////////
  template<typename Pred>
  void erase_edges_if(Pred&& pred)
  {
    this->data()->erase_edges_if(std::forward<Pred>(pred));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all duplicate edges with the same (source, target) pair
  //////////////////////////////////////////////////////////////////////
  void remove_duplicate_edges()
  {
    this->data()->remove_duplicate_edges();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief The number of edges in this base-container.
  //////////////////////////////////////////////////////////////////////
  size_t num_edges(void) const
  {
    if (this->is_edges_loaded())
      return this->data()->num_edges();
    else return 0;
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
    return add_edge_pair(*(this->data()), ed, p);
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
    this->m_active = true;
    if (!this->is_loaded()) {
      this->store_outstanding_commits(gid, f);
    } else {
      f.template operator()<value_type&>(*(data()->find_vertex(gid)));
    }
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
    f.template operator()<value_type&>(*(data()->find_vertex(gid)));
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
    return f.template operator()<value_type&>(*(data()->find_vertex(gid)));
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
    return f.template operator()<value_type const&>(
        *(data()->find_vertex(gid)));
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
    this->m_active = true;
    if (!this->is_vertices_loaded()) {
      this->store_outstanding_commits_cached(gid, f);
      return typename F::result_type();
    } else {
      return f((*(data()->find_vertex(gid))).property());
    }
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
    this->m_active = true;
    if (!this->is_vertices_loaded()) {
      this->store_outstanding_commits_cached(gid, f);
      return typename F::result_type();
    } else {
      return f((*(data()->find_vertex(gid))).property());
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the property of the specified vertex to the given value.
  /// @param gid The descriptor of the vertex.
  /// @param vp The new value of the property.
  //////////////////////////////////////////////////////////////////////
  void vp_set(vertex_descriptor const& gid,
              typename Traits::stored_type const& vp)
  {
    this->m_active = true;
    (*(data()->find_vertex(gid))).property() = vp;
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
    this->m_active = true;
    if (!this->is_vertices_loaded()) {
      this->store_outstanding_commits_cached(gid, f);
    } else {
      f((*(data()->find_vertex(gid))).property());
    }
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
    this->m_active = true;
    for (typename FCont::const_iterator it = fcont.begin(); it != fcont.end();
         ++it)
      it->operator() (data()->find_vertex(it->target())->property());
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
    this->m_active = true;
    this->m_graph_structure_modifications = true;
    typename Traits::container_type::vertex_iterator it;
    adj_edge_iterator eit;
    if (data()->find_edge(ed, it, eit)) {
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
    this->m_active = true;
    this->m_graph_structure_modifications = true;
    typename Traits::container_type::vertex_iterator it;
    adj_edge_iterator eit;
    if (data()->find_edge(ed, it, eit))
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
    this->m_active = true;
    this->m_graph_structure_modifications = true;
    typename Traits::container_type::vertex_iterator it;
    adj_edge_iterator eit;
    if (data()->find_edge(ed, it, eit))
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
    return vertex_reference(accessor_t(this, data()->find_vertex(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the specified vertex.
  /// @param g The descriptor of the vertex.
  /// @return The reference to the specified vertex.
  //////////////////////////////////////////////////////////////////////
  const_vertex_reference make_reference(vertex_descriptor const& gid) const
  {
    return const_vertex_reference(
             const_accessor_t(this, data()->find_vertex(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified vertex.
  /// @param gid The descriptor of the vertex.
  /// @return The iterator to the specified vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator make_iterator(vertex_descriptor const& gid)
  {
    return vertex_iterator(data()->find_vertex(gid), this);
  }

  container_type& container(void)
  {
    return *data();
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

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the id of this base-container.
  //////////////////////////////////////////////////////////////////////
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
    t.member(*m_data);
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
    return this->data()->add_vertex(v.descriptor(), vertex_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a vertex to this base-container. Specialized for when
  /// the vertex has a property.
  /// @param v The vertex object to be added.
  /// @return The descriptor of the new vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex_helper(vertex_type const& v, boost::false_type)
  {
    return this->data()->add_vertex(v.descriptor(), v.property());
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

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the hubs-cache for this base-container. Base-container
  /// must be loaded.
  //////////////////////////////////////////////////////////////////////
  void set_hubs(size_t max_hubs)
  {
    m_hubs_cache.clear();
    size_t abs_max = std::numeric_limits<size_t>::max();
    for (size_t i=0; i<max_hubs; ++i) {
      size_t max_vd = 0;
      size_t curr_max = 1;
      for (auto const& v : *m_data) {
        if (v.size() > curr_max && v.size() < abs_max) {
          max_vd = v.descriptor();
          curr_max = v.size();
        }
      }
      if (curr_max > 10)
        m_hubs_cache.insert(std::make_pair(max_vd, cached_vertex_impl_type()));
      abs_max = curr_max;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Frees the memory used by this base-container.
  //////////////////////////////////////////////////////////////////////
  void free(void)
  {
    this->set_loaded(false);
    if (this->m_data == NULL)
      return;
    delete m_data;
    m_data = NULL;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether this base-container is loaded in memory.
  /// Returns true if vertices and edges are both loaded.
  //////////////////////////////////////////////////////////////////////
  bool is_valid_loaded_state() const
  {
    bool vertices_loaded = ((this->m_data != NULL) &&
                            (this->m_vertices_loaded != false));
    bool edges_loaded = ((this->m_data != NULL) &&
                         (this->m_edge_loaded != false));
    bool valid = (vertices_loaded && edges_loaded) == m_loaded;
    return valid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether this base-container is loaded in memory.
  /// Returns true if vertices and edges are both loaded.
  //////////////////////////////////////////////////////////////////////
  bool is_loaded(void) const
  {
    stapl_assert(is_valid_loaded_state(),
                 "Error is_loaded flags are in an inconsistent state.");
    return (this->m_data != NULL) && (this->m_loaded != false);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether this base-container is loaded in memory.
  /// Returns true if vertices and edges are both loaded.
  //////////////////////////////////////////////////////////////////////
  bool is_fully_loaded(void) const
  {
    stapl_assert(is_valid_loaded_state(),
                 "Error is_loaded flags are in an inconsistent state.");
    return (this->m_data != NULL) && (this->m_loaded != false);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether this base-container is partially loaded in memory.
  /// Returns true if only the vertices loaded, but edges are out-of-core.
  //////////////////////////////////////////////////////////////////////
  bool is_partially_loaded(void) const
  {
    stapl_assert(is_valid_loaded_state(),
                 "Error is_loaded flags are in an inconsistent state.");
    return is_vertices_loaded() != is_edges_loaded();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether this base-container's vertices are loaded.
  //////////////////////////////////////////////////////////////////////
  bool is_vertices_loaded(void) const
  { return (this->m_data != NULL) && (this->m_vertices_loaded != false); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether this base-container's edges are loaded.
  //////////////////////////////////////////////////////////////////////
  bool is_edges_loaded(void) const
  { return (this->m_data != NULL) && (this->m_edge_loaded != false); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets this base-container's loaded state.
  //////////////////////////////////////////////////////////////////////
  void set_loaded(bool b)
  {
    stapl_assert(!(this->m_data == NULL && b),
                 "set-loaded: trying to set loaded for NULL data.");
    stapl_assert(is_valid_loaded_state(),
                 "Error is_loaded flags are in an inconsistent state.");
    this->m_loaded = b;
    this->m_vertices_loaded = b;
    this->m_edge_loaded = b;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets this base-container's loaded state for vertices.
  //////////////////////////////////////////////////////////////////////
  void set_vertices_loaded(bool b)
  {
    stapl_assert(!(this->m_data == NULL && b),
                 "set-loaded: trying to set loaded for NULL data.");
    this->m_vertices_loaded = b;
    m_loaded = b && this->m_edge_loaded ? true : false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets this base-container's loaded state for edges.
  //////////////////////////////////////////////////////////////////////
  void set_edges_loaded(bool b)
  {
    stapl_assert(!(this->m_data == NULL && b),
                 "set-loaded: trying to set loaded for NULL data.");
    this->m_edge_loaded = b;
    m_loaded = b && this->m_vertices_loaded ? true : false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether this base-container is active, i.e., has any
  /// modifications that need to be persisted.
  //////////////////////////////////////////////////////////////////////
  bool is_active(void)
  { return this->m_active; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets this base-container's active state, i.e., if it has any
  /// modifications that need to be persisted.
  //////////////////////////////////////////////////////////////////////
  void set_active(bool active)
  { this->m_active = active; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Class for storing a triplet.
  /// @tparam T Type of the first element of the triplet.
  /// @tparam U Type of the second element of the triplet.
  /// @tparam V Type of the third element of the triplet.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename U, typename V>
  struct triple
  {
    T first;
    U second;
    V third;

    triple() = default;

    triple(T const& t, U const& u, V const& v)
      : first(t), second(u), third(v)
    { }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Loads the outstanding commits to a base container, then
  /// applies those commits to the base container in the order they are
  /// in the file.
  /// @tparam UF Type of the user provided neighbor-operator.
  //////////////////////////////////////////////////////////////////////
  template <typename UF>
  bool load_apply_outstanding_commits(UF const&)
  {
    char filename[30];
    char name_buffer[12];
    sprintf(name_buffer,"%lu",m_cid);
    strcpy(filename, "bc.");
    strcat(filename, name_buffer);
    strcat(filename, ".commits\0");

    FILE* commit_file;
    commit_file = fopen(filename, "rb");
    if (commit_file !=NULL) {

      if (!this->is_loaded()) {
        this->load(false);
      }

      typedef std::pair<vertex_descriptor, UF> buffer_elt_t;
      const size_t size = sizeof(buffer_elt_t);
      buffer_elt_t buffer[commit_load()];

      size_t read_elts = fread(buffer, size, commit_load(), commit_file);
      while (read_elts) {
        for (size_t t=0; t<read_elts; ++t) {
          this->apply(buffer[t].first, buffer[t].second);
        }
        read_elts = fread(buffer, size, commit_load(), commit_file);
      }
      fclose(commit_file);
      std::remove(filename);

      // write-back from hubs cache
      for (auto const& e : m_hubs_cache) {
        this->vp_set(e.first, e.second.property());
      }

      return true;
    }
    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Stores the outstanding commits to a base container. If vertex
  /// exists in cache, applies to that instead. Since cached vertices do not
  /// cache edges, this can only be used for vp_apply().
  /// @tparam UF Type of the user provided neighbor-operator.
  //////////////////////////////////////////////////////////////////////
  template <typename UF>
  void store_outstanding_commits_cached(vertex_descriptor const& vd,
                                        UF const& uf)
  {
    // find the vertex in cache, if found, use the cached value,
    // else write to storage.
    auto it = m_hubs_cache.find(vd);
    if (it != m_hubs_cache.end()) {
     uf.template operator()<cached_vertex_impl_type&>(it->second);
    } else {
      store_outstanding_commits(vd, uf);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Stores the outstanding commits to a base container.
  /// @tparam UF Type of the user provided neighbor-operator.
  //////////////////////////////////////////////////////////////////////
  template <typename UF>
  void store_outstanding_commits(vertex_descriptor const& vd,
                                 UF const& uf)
  {
    typedef std::pair<vertex_descriptor, UF> buffer_elt_t;
    const size_t size = sizeof(buffer_elt_t);

    buffer_elt_t elt = std::make_pair(vd, uf);

    // If we need to flush the commit buffer, flush it first.
    if (m_num_buffered+size >= commit_buffer())
      this->flush_outstanding_commits(false);

    // Stick new values on commit buffer.
    memcpy(m_commit_buffer + m_num_buffered, &elt, sizeof(buffer_elt_t));
    m_num_buffered += sizeof(buffer_elt_t);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Flushes the outstanding commits to a base container.
  //////////////////////////////////////////////////////////////////////
  void flush_outstanding_commits(bool wait_until_finished = true)
  {
    if (m_num_buffered > 0)
    {
      char filename[30];
      char name_buffer[12];
      sprintf(name_buffer,"%lu",m_cid);
      strcpy(filename, "bc.");
      strcat(filename, name_buffer);
      strcat(filename, ".commits\0");

      FILE* commit_file;
      commit_file = fopen(filename, "ab");
      if (commit_file !=NULL)
      {
        size_t written_bytes =
          fwrite(&m_commit_buffer, 1, m_num_buffered, commit_file);

        if (written_bytes != m_num_buffered)
          stapl_assert(false, "Failed writing off-core commit to file.");

        fclose(commit_file);
        std::remove(filename);
        m_num_buffered = 0;
      }
      else
      {
        stapl_assert(false, "Unable to create or open graph commit file.");
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Loads the outstanding edge-commits to a base container, then
  /// adds those edges to the base container in the order they are
  /// in the file.
  //////////////////////////////////////////////////////////////////////
  bool load_apply_outstanding_edges(void)
  {
    char filename[35];
    char name_buffer[12];
    sprintf(name_buffer,"%lu",m_cid);
    strcpy(filename, "bc.");
    strcat(filename, name_buffer);
    strcat(filename, ".outstanding_edges\0");

    FILE* commit_file;
    commit_file = fopen(filename, "rb");
    if (commit_file !=NULL) {

      if (!this->is_loaded())
        this->load();

      typedef triple<bool, edge_descriptor, edge_property> buffer_elt_t;
      const size_t size = sizeof(buffer_elt_t);
      buffer_elt_t buffer[commit_load()];

      size_t read_elts = fread(buffer, size, commit_load(), commit_file);
      while (read_elts) {
        for (size_t t=0; t<read_elts; ++t) {
          this->add_edge(buffer[t].second, buffer[t].third,
                         buffer[t].first);
        }
        read_elts = fread(buffer, size, commit_load(), commit_file);
      }
      fclose(commit_file);
      std::remove(filename);
      this->sort_edges(detail::edge_target_comp());
      return true;
    }
    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Stores the outstanding edge-commits to a base container.
  //////////////////////////////////////////////////////////////////////
  void store_outstanding_edges(edge_descriptor const& ed,
                               edge_property const& ep,
                               bool bidir = false)
  {
    typedef triple<bool, edge_descriptor, edge_property> buffer_elt_t;
    const size_t size = sizeof(buffer_elt_t);

    buffer_elt_t elt(bidir, ed, ep);

    // If we need to flush the commit buffer, flush it first.
    if (m_num_buffered_edges+size >= commit_buffer())
      this->flush_outstanding_edges(false);

    // Stick new values on commit buffer.
    memcpy(m_edges_buffer + m_num_buffered_edges, &elt, sizeof(buffer_elt_t));
    m_num_buffered_edges += sizeof(buffer_elt_t);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Flushes the outstanding commits to a base container.
  //////////////////////////////////////////////////////////////////////
  void flush_outstanding_edges(bool wait_until_finished = true)
  {
    if (m_num_buffered_edges > 0)
    {
      char filename[35];
      char name_buffer[12];
      sprintf(name_buffer,"%lu",m_cid);
      strcpy(filename, "bc.");
      strcat(filename, name_buffer);
      strcat(filename, ".outstanding_edges\0");

      FILE* commit_file;
      commit_file = fopen(filename, "ab");
      if (commit_file !=NULL) {

        size_t written_bytes =
          fwrite(&m_edges_buffer, 1, m_num_buffered_edges, commit_file);

        if (written_bytes != m_num_buffered_edges)
          stapl_assert(false, "Failed writing off-core edge-commits to file.");

        fclose(commit_file);
        std::remove(filename);
        m_num_buffered_edges = 0;
      } else {
        stapl_assert(false, "Unable to create or open graph edge-commit file.");
      }
    }
  }
}; // class graph_base_container_storage_base

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
struct graph_base_container_storage_NME
  : public graph_base_container_storage_base<D, stapl::NONMULTIEDGES,
                                             VertexP, EdgeP, Traits>
{
  typedef typename Traits::domain_type                domain_type;
  typedef graph_base_container_storage_base<
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
  graph_base_container_storage_NME(Domain const& domain, cid_type const& cid)
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
  graph_base_container_storage_NME(Domain const& domain, cid_type const& cid,
                                   value_type const& default_value)
    : base_type(domain, cid, default_value)
  { }

  graph_base_container_storage_NME(
    graph_base_container_storage_NME const& other)
    : base_type(other)
  { }

  graph_base_container_storage_NME() = default;

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
//////////////////////////////////////////////////////////////////////
template<typename Traits,
         graph_attributes MultiType>
struct graph_storage_multiplicity_base_container_selector
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
struct graph_storage_multiplicity_base_container_selector<Traits,
                                                          NONMULTIEDGES>
{
  typedef graph_base_container_storage_NME<Traits::d_type,
                                           typename Traits::vertex_property,
                                           typename Traits::edge_property,
                                           Traits> type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of
/// @ref graph_storage_multiplicity_base_container_selector for MULTIEDGES
/// graph.
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
struct graph_storage_multiplicity_base_container_selector<Traits,
                                                          MULTIEDGES>
{
  typedef graph_base_container_storage_base<Traits::d_type,
                                            MULTIEDGES,
                                            typename Traits::vertex_property,
                                            typename Traits::edge_property,
                                            Traits> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Graph's base-container. Inherits all functionality from either
/// @ref graph_base_container_storage_base or
/// @ref graph_base_container_storage_NME class.
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
struct graph_base_container_storage
  : public graph_storage_multiplicity_base_container_selector<
      Traits, Traits::m_type>::type
{
  typedef graph_base_container_storage<Traits>                  this_type;
  typedef typename Traits::domain_type                          domain_type;
  typedef size_t                                                cid_type;

  typedef typename graph_storage_multiplicity_base_container_selector<
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
  graph_base_container_storage(Domain const& domain, cid_type const& cid)
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
  graph_base_container_storage(Domain const& domain, cid_type const& cid,
                               value_type const& default_value)
    : base_type(domain, cid, default_value) { }

  graph_base_container_storage(graph_base_container_storage const& other)
    : base_type(other)
  { }

  graph_base_container_storage() = default;

}; // struct graph_base_container_storage

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_BASE_CONTAINER_STORAGE_HPP
