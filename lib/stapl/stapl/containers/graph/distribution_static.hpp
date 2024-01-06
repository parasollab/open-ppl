/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_DISTRIBUTION_STATIC_HPP
#define STAPL_CONTAINERS_GRAPH_DISTRIBUTION_STATIC_HPP

#include <boost/bind/bind.hpp>
#include <stapl/containers/graph/graph_accessor.hpp>
#include <stapl/containers/graph/const_graph_accessor.hpp>
#include <stapl/containers/iterators/container_iterator.hpp>
#include <stapl/containers/graph/functional.hpp>
#include <stapl/containers/graph/distribution_base.hpp>
#include <stapl/containers/distribution/static_metadata.hpp>
#include <stapl/containers/distribution/distribution.hpp>
#include <stapl/containers/distribution/operations/base.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Distribution class for the @ref stapl::graph container.
/// @ingroup pgraphDist
///
/// @tparam Container Type of the container that is managing
/// this distribution.
/// @see graph.
/// @note This distribution provides methods for adding and deleting
/// vertices that are not used by the static @ref stapl::graph, but are
/// provided to allow user-defined containers the full
/// @ref dynamic_graph API.
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct graph_distribution_static
  : public detail::graph_distribution_base<
      Container, graph_distribution_static<Container>
    >
{
  using base_type = detail::graph_distribution_base<
    Container, graph_distribution_static<Container>
  >;

public:
  typedef Container                                         container_type;
  typedef typename base_type::directory_type                directory_type;
  typedef typename base_type::container_manager_type container_manager_type;

  typedef typename directory_type::partition_type           partition_type;
  typedef typename directory_type::mapper_type              mapper_type;
  typedef typename container_manager_type::
                     base_container_type                    base_container_type;

  typedef typename directory_type::key_type                 gid_type;
  typedef gid_type                                          index_type;
  typedef typename partition_type::value_type               domain_type;

  typedef typename base_container_type::value_type          value_type;
  typedef typename base_container_type::cid_type            cid_type;
  typedef typename base_container_type::vertex_descriptor   vertex_descriptor;
  typedef typename base_container_type::edge_descriptor     edge_descriptor;
  typedef typename container_traits<container_type>::vertex_property
                                                            vertex_property;
  typedef typename container_traits<container_type>::edge_property
                                                            edge_property;
  typedef typename base_container_type::edgelist_type       edgelist_type;
  typedef typename base_container_type::edge_iterator       edge_iterator;
  typedef typename mapper_type::value_type                  location_type;

  typedef graph_accessor<graph_distribution_static>         accessor_type;
  typedef proxy<value_type, accessor_type>                  reference;

  typedef const_graph_accessor<graph_distribution_static>   const_accessor_type;
  typedef const proxy<value_type, const_accessor_type>      const_reference;

  typedef metadata::static_container_extractor<
    graph_distribution_static
  >                                                         loc_dist_metadata;

  typedef container_iterator<graph_distribution_static,
                             accessor_type>                 iterator;
  typedef const_container_iterator<graph_distribution_static,
                                   const_accessor_type>     const_iterator;
  typedef iterator                                          vertex_iterator;
  typedef const_iterator                              const_vertex_iterator;
  typedef typename reference::adj_edge_iterator             adj_edge_iterator;
  typedef typename const_reference::const_adj_edge_iterator
                                                      const_adj_edge_iterator;

protected:
  bool                    m_is_sorted_edges;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a distribution with the specified directory and
  /// base container manager.
  /// @param directory The specified directory for this distribution.
  /// @param bcmngr The base container manager for this distribution.
  //////////////////////////////////////////////////////////////////////
  graph_distribution_static(directory_type const& directory,
                            container_manager_type const& bcmangr)
    : base_type(directory, bcmangr),
      m_is_sorted_edges(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy construction of this distribution
  ///
  /// @param other Another distribution to copy from
  //////////////////////////////////////////////////////////////////////
  graph_distribution_static(partition_type const& partition,
                            mapper_type const& mapper)
    : base_type(partition, mapper),
      m_is_sorted_edges(false)
  { }

  ////////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with an initial value for elements.
  ///
  /// @param partition Partition used by the container
  /// @param mapper Mapper used by the container
  /// @param default_value The value that the elements in this distribution
  /// will be initialized with
  //////////////////////////////////////////////////////////////////////
  graph_distribution_static(partition_type const& partition,
                            mapper_type const& mapper,
                            value_type const& default_value)
    : base_type(partition, mapper, default_value),
      m_is_sorted_edges(false)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with default constructed elements and a
  /// default mapper.
  ///
  /// @param partition Partition used by the container
  //////////////////////////////////////////////////////////////////////
  graph_distribution_static(partition_type const& partition)
    : base_type(partition),
      m_is_sorted_edges(false)
  { }

  bool is_directed(void) const
  {
    return this->container()->is_directed();
  }

protected:
  bool is_local_sorted_edges(void) const
  { return m_is_sorted_edges; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets is_sorted_edges state to given value.
  /// @param sorted Indicates if the flag should be set to true (sorted)
  /// or false (unsorted/unknown state).
  //////////////////////////////////////////////////////////////////////
  void set_local_sorted_edges(bool sorted)
  { m_is_sorted_edges = sorted; }

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::sort_edges(void)
  //////////////////////////////////////////////////////////////////////
  void sort_edges(void)
  {
    typedef detail::edge_target_location_comp<container_type> comp;
    sort_edges(comp(this->container()));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::sort_edges(Comp const&)
  //////////////////////////////////////////////////////////////////////
  template<typename Comp>
  void sort_edges(Comp const& comp)
  {
    if (!this->is_local_sorted_edges()) {
      for (auto& manager : this->m_container_manager) {
        manager.sort_edges(comp);
      }
      this->set_local_sorted_edges(true);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erases all edges that match a user-defined predicate
  /// @param pred A unary predicate that receives a single edge
  //////////////////////////////////////////////////////////////////////
  template<typename Pred>
  void erase_edges_if(Pred&& pred)
  {
    for (auto& bc: this->m_container_manager) {
      bc.erase_edges_if(std::forward<Pred>(pred));
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all duplicate edges with the same (source, target) pair
  //////////////////////////////////////////////////////////////////////
  void remove_duplicate_edges()
  {
    for (auto& bc: this->m_container_manager) {
      bc.remove_duplicate_edges();
    }
  }

  iterator begin(void)
  {
    return make_iterator(this->directory().partition().global_domain().first());
  }

  const_iterator begin(void) const
  {
    return make_iterator(this->directory().partition().global_domain().first());
  }

  iterator end(void)
  {
    return
      make_iterator(this->directory().partition().global_domain().last()+1);
  }

  const_iterator end(void) const
  {
    return make_iterator(
             this->directory().partition().global_domain().last()+1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::reserve_adjacency(vertex_descriptor const&, size_t)
  //////////////////////////////////////////////////////////////////////
  void reserve_adjacency(vertex_descriptor const& gid, size_t num_adjacents)
  {
    if (!this->m_container_manager.contains_invoke(gid,
                &base_container_type::reserve_adjacency, gid, num_adjacents))
    {
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, vertex_descriptor const& gid, size_t num_adjacents) {
            down_cast<graph_distribution_static&>(d).reserve_adjacency(
              gid, num_adjacents);
          },
          std::placeholders::_1, std::placeholders::_2, num_adjacents),
        gid);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::add_vertex(vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_property const& vp)
  {
    vertex_descriptor new_vd
      = this->m_container_manager.invoke(vertex_descriptor(),
                                         &base_container_type::add_vertex, vp);
    this->directory().register_key(new_vd);
    return new_vd;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::add_vertex(vertex_descriptor const&, vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_descriptor const& gid,
                               vertex_property const& vp)
  {
    this->directory().register_key(gid);

    return this->m_container_manager.invoke(
      gid, &base_container_type::add_vertex, gid, vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::delete_vertex(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(vertex_descriptor const& gid)
  {
    if (auto opt = this->m_container_manager.contains_invoke(gid,
                    &base_container_type::delete_vertex, gid)) {
      return *opt;
    }

    // else
    return this->directory().invoke_where(
      std::bind(
        [](p_object& d, vertex_descriptor const& gid)
          { down_cast<graph_distribution_static&>(d).delete_vertex(gid); },
        std::placeholders::_1, std::placeholders::_2),
      gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::find_vertex(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  iterator find_vertex(vertex_descriptor const& gid)
  {
    return make_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::find_vertex(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  iterator find(vertex_descriptor const& gid)
  {
    return make_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::add_edge(edge_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed)
  {
    return add_edge(ed, edge_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::add_edge(edge_descriptor const&, edge_property const&)
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed, edge_property const& ep)
  {
    promise<edge_descriptor> p;
    auto f = p.get_future();
    add_edge_promise(ed, ep, std::move(p));
    set_local_sorted_edges(false);
    return f.get(); // sync_rmi() equivalent
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::add_edge_async(edge_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  void add_edge_async(edge_descriptor const& ed)
  {
    this->add_edge_async(ed, edge_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::add_edge_async(edge_descriptor const&, edge_property const&)
  /// @param sibling Used to indicate if the edge is the second sibling in an
  /// undirected edge-pair.
  //////////////////////////////////////////////////////////////////////
  void add_edge_async(edge_descriptor const& ed, edge_property const& ep,
                      bool sister = false)
  {
    typedef edge_descriptor (base_container_type::* mem_fun_t)
      (edge_descriptor const&, edge_property const&);

    constexpr mem_fun_t mem_fun = &base_container_type::add_edge;

    // If this is a self edge, do not try to add the reverse edge
    set_local_sorted_edges(false);
    sister |= ed.source() == ed.target();
    if (auto opt = this->m_container_manager.contains_invoke(ed.source(),
                            mem_fun, ed, ep)) {
      // add the correct sister edge for Undirected Graphs.
      if (!sister && !this->is_directed()) {
        edge_descriptor ned = *opt;
        if (ned.id() !=
            std::numeric_limits<typename edge_descriptor::edge_id_type>::max())
        {
          this->add_edge_async(reverse(ned), ep, true);
        }
      }
    } else {
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, edge_descriptor const& ed,
             edge_property const& ep, bool sister) {
               down_cast<graph_distribution_static&>(d).add_edge_async(
                 ed, ep, sister);
          },
          std::placeholders::_1, ed, ep, sister),
          ed.source());
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::insert_edge(edge_descriptor const&, edge_property const&, Comp const&)
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  edge_descriptor insert_edge(edge_descriptor const& ed,
                              edge_property const& ep, Comp const& comp)
  {
    promise<edge_descriptor> p;
    auto f = p.get_future();
    insert_edge_promise(ed, ep, comp, std::move(p));
    set_local_sorted_edges(false);
    return f.get(); // sync_rmi() equivalent
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::insert_edge_async(edge_descriptor const&, edge_property const&)
  /// @param sibling Used to indicate if the edge is the second sibling in an
  /// undirected edge-pair.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_async(edge_descriptor const& ed, edge_property const& ep,
                         Comp const& comp, bool sister = false)
  {
    typedef edge_descriptor (base_container_type::* mem_fun_t)
      (edge_descriptor const&, edge_property const&, Comp const&);

    constexpr mem_fun_t mem_fun = &base_container_type::insert_edge;

    set_local_sorted_edges(false);
    // If this is a self edge, do not try to add the reverse edge
    sister |= ed.source() == ed.target();
    if (auto opt = this->m_container_manager.contains_invoke(ed.source(),
                           mem_fun, ed, ep, comp)) {
      // add the correct sister edge for Undirected Graphs.
      if (!sister && !this->is_directed()) {
        edge_descriptor ned = *opt;
        if (ned.id() !=
            std::numeric_limits<typename edge_descriptor::edge_id_type>::max())
        {
          this->insert_edge_async(reverse(ned), ep, comp, true);
        }
      }
    } else {
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, edge_descriptor const& ed,
             edge_property const& ep, Comp const& comp, bool sister)
          {
             down_cast<graph_distribution_static&>(d).
               insert_edge_async(ed, ep, comp, sister);
           },
           std::placeholders::_1, ed, ep, comp, sister),
        ed.source());
    }
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief used by @ref add_edge method to populate a promise so that
  /// the edge descriptor may be returned for the synchronous method.
  /// @param ed The edge descriptor of the edge being added.
  /// @param ep The property of the edge being added.
  /// @param p The promise object used to return the edge descriptor.
  /// @param sibling Used to indicate if the edge is the second sibling in an
  /// undirected edge-pair.
  //////////////////////////////////////////////////////////////////////
  void add_edge_promise(edge_descriptor const& ed, edge_property const& ep,
                        promise<edge_descriptor> p, bool sister = false)
  {
    typedef edge_descriptor (base_container_type::* mem_fun_t)
      (edge_descriptor const&, edge_property const&);

    constexpr mem_fun_t mem_fun = &base_container_type::add_edge;

    typedef promise<edge_descriptor> promise_type;

    // If this is a self edge, do not try to add the reverse edge
    sister |= ed.source() == ed.target();
    if (auto opt = this->m_container_manager.contains_invoke(ed.source(),
                            mem_fun, ed, ep)) {
      edge_descriptor ned = *opt;
      p.set_value(ned);

      // add the correct sister edge for Undirected Graphs.
      if (!sister && !this->is_directed()) {
        if (ned.id() !=
            std::numeric_limits<typename edge_descriptor::edge_id_type>::max())
        {
          promise_type p1;
          auto f = p1.get_future();
          this->add_edge_promise(reverse(ned), ep, std::move(p1), true);
          f.get(); // sync_rmi() equivalent
        }
      }
    } else {
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, edge_descriptor const& ed, edge_property const& ep,
             promise_type& p, bool sister)
          {
             down_cast<graph_distribution_static&>(d).
               add_edge_promise(ed, ep, std::move(p), sister);
          },
          std::placeholders::_1, ed, ep, std::move(p), sister),
        ed.source());
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief used by @ref insert_edge method to populate a promise so that
  /// the edge descriptor may be returned for the synchronous method.
  /// @param ed The edge descriptor of the edge being added.
  /// @param ep The property of the edge being added.
  /// @param comp Passed internal to std::lower_bound to find the spot to insert
  /// the new edge.
  /// @param p The promise object used to return the edge descriptor.
  /// @param sibling Used to indicate if the edge is the second sibling in an
  /// undirected edge-pair.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_promise(edge_descriptor const& ed, edge_property const& ep,
              Comp const& comp, promise<edge_descriptor> p, bool sister = false)
  {
    typedef promise<edge_descriptor> promise_type;
    typedef edge_descriptor (base_container_type::* mem_fun_t)
      (edge_descriptor const&, edge_property const&, Comp const&);

    constexpr mem_fun_t mem_fun = &base_container_type::insert_edge;

    // If this is a self edge, do not try to add the reverse edge
    sister |= ed.source() == ed.target();
    if (auto opt = this->m_container_manager.contains_invoke(ed.source(),
                            mem_fun, ed, ep, comp)) {
      edge_descriptor ned = *opt;
      p.set_value(ned);

      // add the correct sister edge for Undirected Graphs.
      if (!sister && !this->is_directed()) {
        if (ned.id() !=
            std::numeric_limits<typename edge_descriptor::edge_id_type>::max())
        {
          promise_type p1;
          auto f = p1.get_future();
          this->insert_edge_promise(
            reverse(ned), ep, comp, std::move(p1), true);
          f.get(); // sync_rmi() equivalent
        }
      }
    } else {
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, edge_descriptor const& ed, edge_property const& ep,
             Comp const& comp, promise_type& p, bool sister)
          {
            down_cast<graph_distribution_static&>(d).
              insert_edge_promise(ed, ep, comp, std::move(p), sister);
          },
          std::placeholders::_1, ed, ep, comp, std::move(p), sister),
        ed.source());
    }
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::delete_edge(edge_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  void delete_edge(edge_descriptor const& ed)
  {
    if (!this->m_container_manager.contains_invoke(ed.source(),
                                    &base_container_type::delete_edge, ed)) {
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, edge_descriptor const& ed)
            { down_cast<graph_distribution_static&>(d).delete_edge(ed); },
          std::placeholders::_1, ed),
        ed.source());
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::clear()
  /// @note This method assumes being called in SPMD (by all locations).
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->m_container_manager.clear();
    this->directory().reset();
    this->advance_epoch();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::num_local_edges()
  //////////////////////////////////////////////////////////////////////
  size_t num_local_edges(void) const
  {
    return this->m_container_manager.num_edges();
  }

  /////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of local self edges in the base container.
  /// @return size_t total number of self edges for all local vertices.
  //////////////////////////////////////////////////////////////////////
  size_t num_local_self_edges(void) const
  {
    return this->m_container_manager.num_self_edges();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::num_edges()
  //////////////////////////////////////////////////////////////////////
  size_t num_edges(void) const
  {
    return
      unordered::sync_reduce_rmi(std::plus<size_t>(), this->get_rmi_handle(),
                                 &graph_distribution_static::num_local_edges);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of self edges in the pGraph.
  /// @return size_t total number of self edges for all local vertices.
  //////////////////////////////////////////////////////////////////////
  size_t num_self_edges(void) const
  {
    return
      unordered::sync_reduce_rmi(std::plus<size_t>(), this->get_rmi_handle(),
                              &graph_distribution_static::num_local_self_edges);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::num_edges_collective()
  //////////////////////////////////////////////////////////////////////
  size_t num_edges_collective(void) const
  {
    return allreduce_rmi(std::plus<size_t>(),
                         this->get_rmi_handle(),
                         &graph_distribution_static::num_local_edges).get();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of self edges in the pGraph.
  /// @warning This method is a blocking, collective version of num_self_edges.
  /// Use when calling num_self_edges from all locations, which is faster than
  /// calling @ref num_self_edges() above.
  /// @return size_t number of self edges in the pGraph.
  //////////////////////////////////////////////////////////////////////
  size_t num_self_edges_collective(void) const
  {
    return allreduce_rmi(std::plus<size_t>(),
                         this->get_rmi_handle(),
                        &graph_distribution_static::num_local_self_edges).get();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies functors to respective vertex-properties on this
  /// location. All elements must be local. For internal use by Aggregators.
  /// @param f Container of functors to be applied. Each element of f
  /// should provide a target() method that returns the descriptor of the
  /// target vertex.
  /// Each element of f is given the property of its corresponding vertex.
  //////////////////////////////////////////////////////////////////////
  template<typename FCont>
  void aggregate_vp_apply_async(FCont const& fcont)
  {
    this->m_container_manager.begin()->aggregate_vp_apply(fcont);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::aggregate_vp_apply_async(size_t const&, FCont const&)
  //////////////////////////////////////////////////////////////////////
  template<typename FCont>
  void aggregate_vp_apply_async(size_t loc, FCont const& fcont)
  {
    async_rmi(loc,
              this->get_rmi_handle(),
              (void (graph_distribution_static::*)(const FCont&))
              &graph_distribution_static::aggregate_vp_apply_async,
              fcont);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a functor on this location.
  /// All elements must be local. For internal use by Aggregators.
  /// @param v Container of elements to be passed-in to f.
  /// @param f Functor to be applied to each element of v on the desired
  /// location. f is passed an element of v as well as a pointer to the pGraph.
  //////////////////////////////////////////////////////////////////////
  template<typename Cont, typename F>
  void aggregate_async(Cont const& cont, F const& f)
  {
    for (typename Cont::const_iterator it = cont.begin();
         it != cont.end();
         ++it)
      f(this, *it);
  }


  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::aggregate_async(size_t const&, Const const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<typename Cont, typename F>
  void aggregate_async(size_t loc, Cont const& cont, F const& f)
  {
    if (loc == this->get_location_id())
      aggregate_async(cont, f);
    else
      async_rmi(loc,
                this->get_rmi_handle(),
                (void (graph_distribution_static::*)(Cont const&, F const&))
                &graph_distribution_static::aggregate_async,
                cont, f);
  }



  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::vp_apply_async(vertex_descriptor const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void vp_apply_async(vertex_descriptor const& gid, F const& f)
  {
    this->apply_set(gid, detail::extract_property_apply_set<F>(f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::vp_apply(gid_type const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  typename F::result_type vp_apply(gid_type const& gid, F const& f)
  {
    return this->apply_get(gid, detail::extract_property_apply<F>(f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::vp_apply(gid_type const&, F const&) const
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  typename F::result_type vp_apply(gid_type const& gid, F const& f) const
  {
    return this->apply_get(gid, detail::extract_property_apply<F>(f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::ep_apply_async(edge_descriptor const&, Functor const&)
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void ep_apply_async(edge_descriptor const& ed, Functor const& f)
  {
    if (this->m_container_manager.contains(ed.source()))
      this->m_container_manager.ep_apply(ed, f);
    else
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, edge_descriptor const& ed, Functor const& f)
            { down_cast<graph_distribution_static&>(d).ep_apply_async(ed, f); },
          std::placeholders::_1, ed, f),
        ed.source());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::ep_apply(edge_descriptor const&, Functor const&)
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type ep_apply(edge_descriptor const& ed,
                                         Functor const& f)
  {
    promise<typename Functor::result_type> p;
    auto ft = p.get_future();
    ep_apply_promise(ed, f, std::move(p));
    return ft.get(); // sync_rmi() equivalent
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function to the specified edge and populates a
  /// promise with the result. Used by ep_apply.
  /// @param ed Descriptor of the edge.
  /// @param f Functor to be applied to the target edge's property.
  /// @return p Promise storing the result of applying f to the edge's property.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void ep_apply_promise(edge_descriptor const& ed, Functor const& f,
                        promise<typename Functor::result_type> p)
  {
    typedef typename Functor::result_type result_type;

    if (this->m_container_manager.contains(ed.source()))
    {
      result_type r = this->m_container_manager.ep_apply_get(ed, f);
      p.set_value(r);
    } else {
      typedef promise<result_type> promise_type;
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, edge_descriptor const& ed, Functor const& f,
             promise_type& p)
          {
            down_cast<graph_distribution_static&>(d).ep_apply_promise(
              ed, f, std::move(p));
          },
          std::placeholders::_1, ed, f, std::move(p)),
        ed.source());
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph_base_container_base::ep_find_apply(edge_descriptor const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  bool ep_find_apply(edge_descriptor const& ed,
                     Functor const& f)
  {
    promise<bool> p;
    auto ft = p.get_future();
    ep_find_apply_promise(ed, f, std::move(p));
    return ft.get(); // sync_rmi() equivalent
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the given functor on the property of the specified edge,
  /// if it exists. Populates a promise variable with true if the edge was
  /// found, or false otherwise.
  /// @param ed Descriptor of the edge.
  /// @param f Functor to be applied to the target edge's property.
  /// @return p Promise storing the result of applying f to the edge's property.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void ep_find_apply_promise(edge_descriptor const& ed, Functor const& f,
                             promise<bool> p)
  {
    if (this->m_container_manager.contains(ed.source())) {
      bool r = this->m_container_manager.ep_find_apply(ed, f);
      p.set_value(r);
    } else {
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, edge_descriptor const& ed, Functor const& f,
             promise<bool>& p)
          {
            down_cast<graph_distribution_static&>(d).ep_find_apply_promise(
              ed, f, std::move(p));
          },
          std::placeholders::_1, ed, f, std::move(p)),
        ed.source());
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::make_iterator(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return iterator(this, gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::make_iterator(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  const_iterator make_iterator(gid_type const& gid) const
  {
    return const_iterator(this, gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an iterator to the specified gid, which is restricted
  /// to the provided domain.
  /// @param domain The domain to which the iterator will be restricted.
  /// @param gid The gid of the vertex to which the iterator will point.
  /// @return The iterator pointing to the specified gid, restricted to the
  /// given domain.
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(domain_type const& domain, gid_type const& gid)
  {
    return iterator(this,domain,gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::make_reference(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  reference make_reference(index_type const& gid)
  {
    return reference(accessor_type(this, gid));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::make_reference(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  const_reference make_reference(index_type const& gid) const
  {
    return const_reference(const_accessor_type(this, gid));
  }

  typedef metadata_entry<domain_type,base_container_type*> dom_info_type;
  typedef std::vector<dom_info_type>                       local_return_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the metadata information for the base-containers on this
  /// location.
  /// @see metadata_entry.
  ////////////////////////////////////////////////////////////////////////
  local_return_type metadata(void)
  {
    local_return_type v;
    typename container_manager_type::iterator bcit =
      this->container_manager().begin();
    typename container_manager_type::iterator bcend =
      this->container_manager().end();
    for (; bcit != bcend; ++bcit)
    {
      base_container_type& bc = *bcit;
      if (!bc.domain().empty())
        v.push_back(dom_info_type(
          bc.cid(), bc.domain(), &bc,
          LQ_CERTAIN, get_affinity(),
          this->get_rmi_handle(), this->get_location_id()
        ));
    }
    return v;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the global number of base-containers in this distribution.
  ////////////////////////////////////////////////////////////////////////
  size_t num_base_containers(void)
  {
    return this->partition().size();
  }
}; // struct graph_distribution_static

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_DISTRIBUTION_STATIC_HPP
