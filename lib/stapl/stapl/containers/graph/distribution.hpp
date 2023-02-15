/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_DISTRIBUTION_HPP
#define STAPL_CONTAINERS_GRAPH_DISTRIBUTION_HPP

#include <boost/bind/bind.hpp>
#include <stapl/containers/graph/functional.hpp>
#include <stapl/containers/graph/distribution_base.hpp>
#include <stapl/containers/distribution/distribution.hpp>
#include <stapl/containers/distribution/operations/base.hpp>
#include <stapl/containers/graph/distribution_traits.hpp>
#include <stapl/containers/distribution/graph_metadata.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Distribution class for the @ref dynamic_graph container.
/// @ingroup pgraphDist
///
/// @tparam Container Type of the container that is managing
/// this distribution.
/// @see dynamic_graph
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct graph_distribution
  : public detail::graph_distribution_base<
      Container, graph_distribution<Container>
    >,
    public operations::migratable<graph_distribution<Container>>
{
  using base_type = detail::graph_distribution_base<
    Container, graph_distribution<Container>
  >;

public:
  typedef Container                                         container_type;
  typedef typename base_type::directory_type                directory_type;
  typedef typename base_type::container_manager_type container_manager_type;

  typedef typename directory_type::partition_type           partition_type;
  typedef typename directory_type::mapper_type              mapper_type;
  typedef typename container_manager_type::
                   base_container_type                      base_container_type;

  typedef typename directory_type::key_type                 gid_type;
  typedef gid_type                                          index_type;
  typedef typename graph_distribution_traits<
                     graph_distribution, container_type>::
                   domain_type                              domain_type;

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

  typedef graph_accessor<graph_distribution>                accessor_type;
  typedef proxy<value_type, accessor_type>                  reference;

  typedef const_graph_accessor<graph_distribution>          const_accessor_type;
  typedef const proxy<value_type, const_accessor_type>      const_reference;


  typedef graph_metadata<graph_distribution>                loc_dist_metadata;

  typedef container_iterator<graph_distribution,
                             accessor_type>                 iterator;
  typedef const_container_iterator<graph_distribution,
                                   const_accessor_type>     const_iterator;
  typedef iterator                                          vertex_iterator;
  typedef const_iterator                              const_vertex_iterator;
  typedef typename reference::adj_edge_iterator             adj_edge_iterator;
  typedef typename const_reference::const_adj_edge_iterator
                                                      const_adj_edge_iterator;

protected:
  domain_type             m_domain;
  bool                    m_is_sorted_edges;

private:
  void register_local_keys(void)
  {
    typedef typename container_manager_type::iterator bc_iterator;
    typedef typename container_manager_type::base_container_type bc_type;
    typedef typename bc_type::iterator vertex_iterator;
    bc_iterator bc_it = this->m_container_manager.begin();
    bc_iterator bc_end = this->m_container_manager.end();
    for (; bc_it != bc_end; ++bc_it)
    {
      if (bc_it->size() != 0)
      {
        for (vertex_iterator vi = (*bc_it).begin(); vi != (*bc_it).end(); ++vi)
        {
          this->directory().register_key((*vi).descriptor());
        }
      }
    }
  }
public:
  graph_distribution(graph_distribution const& other)
    : base_type(other),
      m_domain(*this),
      m_is_sorted_edges(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with default constructed elements.
  ///
  /// @param partition Partition used by the container
  /// @param mapper Mapper used by the container
  //////////////////////////////////////////////////////////////////////
  graph_distribution(partition_type const& partition, mapper_type const& mapper)
    : base_type(partition, mapper),
      m_domain(*this),
      m_is_sorted_edges(false)
  {
    register_local_keys();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with an initial value for elements.
  ///
  /// @param partition Partition used by the container
  /// @param mapper Mapper used by the container
  /// @param default_value The value that the elements in this distribution
  /// will be initialized with
  //////////////////////////////////////////////////////////////////////
  graph_distribution(partition_type const& partition, mapper_type const& mapper,
                     value_type const& default_value)
    : base_type(partition, mapper, default_value),
      m_domain(*this),
      m_is_sorted_edges(false)
  {
    register_local_keys();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with default constructed elements and a
  /// default mapper.
  ///
  /// @param partition Partition used by the container
  //////////////////////////////////////////////////////////////////////
  graph_distribution(partition_type const& partition)
    : base_type(partition),
      m_domain(*this),
      m_is_sorted_edges(false)
  {
    register_local_keys();
  }

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
  /// @copydoc directed_graph_base::sort_edges(void)
  //////////////////////////////////////////////////////////////////////
  void sort_edges(void)
  {
    typedef detail::edge_target_location_comp<container_type> comp;
    sort_edges(comp(this->container()));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::sort_edges(Comp const&)
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

  iterator begin(void)
  {
    return make_iterator(this->m_domain.first());
  }

  const_iterator begin(void) const
  {
    return make_iterator(this->m_domain.first());
  }

  iterator end(void)
  {
    return make_iterator(index_bounds<gid_type>::invalid());
  }

  const_iterator end(void) const
  {
    return make_iterator(index_bounds<gid_type>::invalid());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::add_vertex(vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_property const& vp)
  {
    vertex_descriptor new_vd = this->m_container_manager.min_invoke(
      boost::bind(std::less<std::size_t>(),
        boost::bind(&base_container_type::size, _1),
        boost::bind(&base_container_type::size, _2)
      ),
      &base_container_type::add_vertex, vp
    );

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
    return this->m_container_manager.min_invoke(
      boost::bind(std::less<std::size_t>(),
        boost::bind(&base_container_type::size, _1),
        boost::bind(&base_container_type::size, _2)
      ),
      &base_container_type::add_vertex, gid, vp
    );
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief @copybrief add_vertex
  ///
  /// This will only be executed on the home location of the gid, as it
  /// is at this point that we can make the decision of insertion or
  /// application.
  ///
  /// @param gid The gid of the vertex to be added.
  /// @param vp The vertex property of the vertex to be added.
  /// @param f The functor to apply if the element already exists
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void try_insert(vertex_descriptor const& gid, vertex_property const& vp,
                  Functor const& f)
  {
    // Vertex exists, apply function f on it.
    if (this->directory().is_registered_local(gid)) {
      this->apply_set(gid, f);
    } else {  // Vertex doesn't exist, add it.
      this->directory().register_key(gid);

      this->m_container_manager.min_invoke(
        boost::bind(std::less<std::size_t>(),
          boost::bind(&base_container_type::size, _1),
          boost::bind(&base_container_type::size, _2)
        ),
        &base_container_type::add_vertex, gid, vp
      );
    }
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::add_vertex(vertex_descriptor const&, vertex_property const&, Functor const&)
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void add_vertex(vertex_descriptor const& gid, vertex_property const& vp,
                  Functor const& f)
  {
    // Go to the location where the vertex should be stored (if it doesn't
    // exist), or is actually stored (if it exists).

    // Unsure if registered at this point, try to check home;
    // can't use invoke_where as it will potentially queue indefinitely
    std::pair<location_type, loc_qual> home =
      this->directory().key_mapper()(gid);

    if (home.first == this->get_location_id()) {
      try_insert(gid, vp, f);
      return;
    } else if (home.second == LQ_CERTAIN) {
      async_rmi(home.first, this->get_rmi_handle(),
                &graph_distribution::template try_insert<Functor>, gid, vp, f);
      return;
    }
    // else
    async_rmi(home.first, this->get_rmi_handle(),
              &graph_distribution::template add_vertex<Functor>, gid, vp, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::add_vertex_uniform(vertex_property const&)
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex_uniform(vertex_property const& vp)
  {
    const vertex_descriptor gid =  this->m_container_manager.min_invoke(
      boost::bind(std::less<std::size_t>(),
        boost::bind(&base_container_type::size, _1),
        boost::bind(&base_container_type::size, _2)
      ),
      &base_container_type::next_free_descriptor
    );

    const location_type dest = gid % this->get_num_locations();

    typedef vertex_descriptor(graph_distribution::* fn_t)
      (vertex_descriptor const&, vertex_property const& vp);

    if (dest == this->get_location_id()) {
      add_vertex(gid, vp);
    } else {
      async_rmi(dest, this->get_rmi_handle(),
                (fn_t) &graph_distribution::add_vertex, gid, vp);
    }

    return gid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc dynamic_graph::delete_vertex(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  void delete_vertex(vertex_descriptor const& gid)
  {
    if (this->m_container_manager.contains(gid))
    {
      const bool b = this->m_container_manager.invoke(
        gid, &base_container_type::delete_vertex, gid);

      if (b)
        this->directory().unregister_key(gid);

      return;
    }

    // else
    this->directory().invoke_where(
      std::bind(
        [](p_object& d, vertex_descriptor const& gid)
          { down_cast<graph_distribution&>(d).delete_vertex(gid); },
        std::placeholders::_1, std::placeholders::_2),
      gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::find_vertex(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  iterator find_vertex(vertex_descriptor const& gid)
  {
    return make_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::find_vertex(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  iterator find(vertex_descriptor const& gid)
  {
    return make_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge with given descriptor and property.
  /// @warning Synchronous. Edge is added before returning.
  /// @param ed Descriptor of the desired edge.
  /// @return edge_descriptor of the added edge.
  /// The id of the edge_descriptor returned is set to
  /// numeric_limits<size_t>::max() if the edge was not added.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed)
  {
    return add_edge(ed, edge_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::add_edge(edge_descriptor const&, edge_property const&)
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
  /// @brief Adds an edge with given descriptor.
  ///
  /// The edge is added asynchronously and method returns immediately.
  /// Edge is not guaranteed to be added until after a global synchronization.
  /// @param ed Descriptor of the desired edge.
  //////////////////////////////////////////////////////////////////////
  void add_edge_async(edge_descriptor const& ed)
  {
    this->add_edge_async(ed, edge_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::add_edge_async(edge_descriptor const&, edge_property const&)
  /// @param sibling Used to indicate if the edge is the second sibling in an
  /// undirected edge-pair.
  //////////////////////////////////////////////////////////////////////
  void add_edge_async(edge_descriptor const& ed, edge_property const& ep,
                      bool sibling = false)
  {
    set_local_sorted_edges(false);

    typedef edge_descriptor (base_container_type::* mem_fun_t)
      (edge_descriptor const&, edge_property const&);

    constexpr mem_fun_t mem_fun = &base_container_type::add_edge;

    // If this is a self edge, do not try to add the reverse edge
    sibling |= ed.source() == ed.target();
    if (auto opt = this->m_container_manager.contains_invoke(
                     ed.source(), mem_fun, ed, ep)) {
      // add the correct sibling edge for Undirected Graphs.
      if (!sibling && !this->is_directed()) {
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
             edge_property const& ep, bool sibling)
          {
            down_cast<graph_distribution&>(d).
              add_edge_async(ed, ep, sibling);
          },
          std::placeholders::_1, ed, ep, sibling),
        ed.source());
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::insert_edge(edge_descriptor const&, Comp const&)
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  edge_descriptor insert_edge(edge_descriptor const& ed, Comp const& comp)
  {
    return insert_edge(ed, edge_property(), comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::insert_edge(edge_descriptor const&, edge_property const&, Comp const&)
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
  /// @brief Inserts an edge with given descriptor.
  ///
  /// The edge is inserted asynchronously and method returns immediately.
  /// Edge is not guaranteed to be added until after a global synchronization.
  /// @param ed Descriptor of the desired edge.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_async(edge_descriptor const& ed, Comp const& comp)
  {
    this->insert_edge_async(ed, edge_property(), comp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::insert_edge_async(edge_descriptor const&, edge_property const&)
  /// @param sibling Used to indicate if the edge is the second sibling in an
  /// undirected edge-pair.
  //////////////////////////////////////////////////////////////////////
  template <typename Comp>
  void insert_edge_async(edge_descriptor const& ed, edge_property const& ep,
                         Comp const& comp, bool sibling = false)
  {
    // If this is a self edge, do not try to add the reverse edge
    sibling |= ed.source() == ed.target();
    set_local_sorted_edges(false);
    if (auto opt = this->m_container_manager.contains_invoke(ed.source(),
                               &base_container_type::template insert_edge<Comp>,
                               ed, ep, comp)) {
      // add the correct sibling edge for Undirected Graphs.
      if (!sibling && !this->is_directed()) {
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
             edge_property const& ep, Comp const& comp, bool sibling)
          {
             down_cast<graph_distribution&>(d).
               insert_edge_async(ed, ep, comp, sibling);
           },
           std::placeholders::_1, ed, ep, comp, sibling),
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
                        promise<edge_descriptor> p, bool sibling = false)
  {
    typedef edge_descriptor (base_container_type::* mem_fun_t)
      (edge_descriptor const&, edge_property const&);

    constexpr mem_fun_t mem_fun = &base_container_type::add_edge;

    typedef promise<edge_descriptor> promise_type;

    // If this is a self edge, do not try to add the reverse edge
    sibling |= ed.source() == ed.target();

    if (auto opt = this->m_container_manager.contains_invoke(
                     ed.source(), mem_fun, ed, ep)) {
      edge_descriptor ned = *opt;
      p.set_value(ned);

      // add the correct sibling edge for Undirected Graphs.
      if (!sibling && !this->is_directed()) {
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
             promise_type& p, bool sibling)
          {
             down_cast<graph_distribution&>(d).add_edge_promise(
               ed, ep, std::move(p), sibling);
          },
          std::placeholders::_1, ed, ep, std::move(p), sibling),
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
            Comp const& comp, promise<edge_descriptor> p, bool sibling = false)
  {
    typedef promise<edge_descriptor> promise_type;

    // If this is a self edge, do not try to add the reverse edge
    sibling |= ed.source() == ed.target();
    if (auto opt = this->m_container_manager.contains_invoke(ed.source(),
                               &base_container_type::template insert_edge<Comp>,
                               ed, ep, comp)) {
      edge_descriptor ned = *opt;
      p.set_value(ned);

      // add the correct sibling edge for Undirected Graphs.
      if (!sibling && !this->is_directed()) {
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
            Comp const& comp, promise_type& p, bool sibling)
          {
            down_cast<graph_distribution&>(d).insert_edge_promise(
              ed, ep, comp, std::move(p), sibling);
          },
          std::placeholders::_1, ed, ep, comp, std::move(p), sibling),
        ed.source());
    }
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::delete_edge(edge_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  void delete_edge(edge_descriptor const& ed)
  {
    if (!this->m_container_manager.contains_invoke(ed.source(),
                                       &base_container_type::delete_edge, ed)) {
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, edge_descriptor const& ed)
            { down_cast<graph_distribution&>(d).delete_edge(ed); },
          std::placeholders::_1, ed),
        ed.source());
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::clear()
  /// @note This method assumes being called in SPMD (by all locations).
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->m_container_manager.clear();
    this->directory().reset();
    this->advance_epoch();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::num_local_edges()
  //////////////////////////////////////////////////////////////////////
  size_t num_local_edges(void) const
  {
    return this->m_container_manager.num_edges();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of self edges in the pGraph.
  /// This is a blocking method.
  /// @return size_t total number of self edges for all local vertices.
  //////////////////////////////////////////////////////////////////////
  size_t num_local_self_edges(void) const
  {
    return this->m_container_manager.num_self_edges();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::num_edges()
  //////////////////////////////////////////////////////////////////////
  size_t num_edges(void) const
  {
    return
      unordered::sync_reduce_rmi(std::plus<size_t>(), this->get_rmi_handle(),
                                 &graph_distribution::num_local_edges);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of self edges in the pGraph.
  /// This must be used when not all locations are calling num_self_edges.
  /// For a faster collective, use @ref num_self_edges_collective() below.
  /// @return size_t number of self edges in the pGraph.
  //////////////////////////////////////////////////////////////////////
  size_t num_self_edges(void) const
  {
    return
      unordered::sync_reduce_rmi(std::plus<size_t>(), this->get_rmi_handle(),
                                 &graph_distribution::num_local_self_edges);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::num_edges_collective()
  //////////////////////////////////////////////////////////////////////
  size_t num_edges_collective(void) const
  {
    return allreduce_rmi(std::plus<size_t>(),
                         this->get_rmi_handle(),
                         &graph_distribution::num_local_edges).get();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of self edges in the pGraph.
  /// @warning This method is blocking, collective version of num_self_edges.
  /// Use when calling num-edges from all locations, which is faster than
  /// calling @ref num_self_edges() above.
  /// @return size_t number of self edges in the pGraph.
  //////////////////////////////////////////////////////////////////////
  size_t num_self_edges_collective(void) const
  {
    return allreduce_rmi(std::plus<size_t>(),
                         this->get_rmi_handle(),
                         &graph_distribution::num_local_self_edges).get();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Applies functors to respective vertex-properties on this
  /// location. All elements must be local. For internal use by Aggregators.
  /// @param fcont Container of functors to be applied. Each element of f
  /// should provide a target() method that returns the descriptor of the
  /// target vertex.
  /// Each element of f is given the property of its corresponding vertex.
  //////////////////////////////////////////////////////////////////////
  template<typename FCont>
  void aggregate_vp_apply_async(FCont const& fcont)
  {
    for (typename FCont::const_iterator it = fcont.begin();
         it != fcont.end();
         ++it)
      this->vp_apply(it->target(), *it);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::aggregate_vp_apply_async(size_t const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<typename FCont>
  void aggregate_vp_apply_async(size_t loc, FCont const& fcont)
  {
    async_rmi(loc,
              this->get_rmi_handle(),
              (void (graph_distribution::*)
                (FCont const&))&graph_distribution::aggregate_vp_apply_async,
              fcont);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a functor on this location.
  /// All elements must be local. For internal use by Aggregators.
  /// @param cont Container of elements to be passed-in to f.
  /// @param f Functor to be applied to each element of cont on the desired
  /// location. f is passed an element of cont as well as a pointer to the
  /// pGraph.
  //////////////////////////////////////////////////////////////////////
  template<typename Cont, typename F>
  void aggregate_async(Cont const& cont, F const& f)
  {
    for (typename Cont::const_iterator it = cont.begin(); it != cont.end();
         ++it)
      f(this, *it);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::aggregate_async(size_t const&, Cont const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<typename Cont, typename F>
  void aggregate_async(size_t loc, Cont const& cont, F const& f)
  {
    if (loc == this->get_location_id())
      aggregate_async(cont, f);
    else
      async_rmi(loc,
                this->get_rmi_handle(),
                (void (graph_distribution::*)(Cont const&, F const&))
                &graph_distribution::aggregate_async, cont, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a functor on this location.
  ///
  /// All elements must be local. For internal use by Aggregators.
  /// @param v Container of elements to be passed-in to f.
  /// @param f Functor to be applied to each element of v on the desired
  /// location. f is passed an element of v.
  //////////////////////////////////////////////////////////////////////
  template<typename Cont, typename F>
  void aggregate_apply_async(Cont const& cont, F const& f)
  {
    using mem_fun_t
      = void (base_container_type::*)(vertex_descriptor const&, F const&);

    constexpr mem_fun_t mem_fun = &base_container_type::apply;

    for (auto const& edge : cont) {
      auto const target = edge.target();

      auto opt
        = this->m_container_manager.contains_invoke(target, mem_fun, target, f);

      // If this vertex is not at the home location, use the directory to route
      // the request to where the vertex is
      if (!opt)
        this->apply_set(target, f);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::aggregate_apply_async(size_t const&, Const const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<typename Cont, typename F>
  void aggregate_apply_async(size_t loc, Cont const& cont, F const& f)
  {
    if (loc == this->get_location_id())
      aggregate_apply_async(cont, f);
    else
      // TODO(afidel): Maybe add another method for ordered async?
      unordered::async_rmi(
        loc,
        this->get_rmi_handle(),
        (void (graph_distribution::*)(Cont const&, F const&))
          & graph_distribution::aggregate_apply_async,
        cont,
        f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc graph::aggregate_apply_async(size_t const&, Const const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<typename Cont, typename F>
  void guarded_aggregate_apply_async(size_t loc, Cont const& cont, F const& f)
  {
    if (!this->try_lock()) {
      unordered::async_rmi(
        loc,
        this->get_rmi_handle(),
        (void (graph_distribution::*)(size_t, Cont const&, F const&))
          & graph_distribution::guarded_aggregate_apply_async,
        loc,
        cont,
        f);

      return;
    }

    if (loc == this->get_location_id())
      aggregate_apply_async(cont, f);
    else
      unordered::async_rmi(
        loc,
        this->get_rmi_handle(),
        (void (graph_distribution::*)(Cont const&, F const&))
          & graph_distribution::aggregate_apply_async,
        cont,
        f);

    this->unlock();
  }
  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::vp_apply_async(vertex_descriptor const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void vp_apply_async(vertex_descriptor const& gid, F const& f)
  {
    this->apply_set(gid, detail::extract_property_apply_set<F>(f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::vp_apply(vertex_descriptor const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  typename F::result_type vp_apply(gid_type const& gid, F const& f)
  {
    return this->apply_get(gid, detail::extract_property_apply<F>(f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::vp_apply(vertex_descriptor const&, F const&) const
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  typename F::result_type vp_apply(gid_type const& gid, F const& f) const
  {
    return this->apply_get(gid, detail::extract_property_apply<F>(f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::ep_apply_async(edge_descriptor const&, F const&)
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void ep_apply_async(edge_descriptor const& ed, Functor const& f)
  {
    if (this->m_container_manager.contains(ed.source()))
      this->m_container_manager.invoke(
        ed.source(),
        &base_container_type::template ep_apply<Functor>, ed, f);
    else
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, edge_descriptor const& ed, Functor const& f)
            { down_cast<graph_distribution&>(d).ep_apply_async(ed, f); },
          std::placeholders::_1, ed, f),
        ed.source());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::ep_apply(edge_descriptor const&, F const&)
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
  /// @param p Promise storing the result of applying f to the edge's property.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void ep_apply_promise(edge_descriptor const& ed, Functor const& f,
                        promise<typename Functor::result_type> p)
  {
    typedef typename Functor::result_type result_type;

    if (this->m_container_manager.contains(ed.source())) {
      result_type r =
        this->m_container_manager.invoke(
          ed.source(),
          &base_container_type::template ep_apply_get<Functor>, ed, f);
      p.set_value(r);
    } else {
      typedef promise<result_type> promise_type;
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, edge_descriptor const& ed, Functor const& f,
             promise_type& p)
          {
            down_cast<graph_distribution&>(d).ep_apply_promise(
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
  /// @param p Promise storing the result of applying f to the edge's property.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void ep_find_apply_promise(edge_descriptor const& ed, Functor const& f,
                             promise<bool> p)
  {
    if (this->m_container_manager.contains(ed.source())) {
      bool r =
        this->m_container_manager.invoke(
          ed.source(),
          &base_container_type::template ep_find_apply<Functor>, ed, f);
      p.set_value(r);
    } else {
      this->directory().invoke_where(
        std::bind(
          [](p_object& d, edge_descriptor const& ed, Functor const& f,
             promise<bool>& p)
          {
            down_cast<graph_distribution&>(d).ep_find_apply_promise(
              ed, f, std::move(p));
          },
          std::placeholders::_1, ed, f, std::move(p)),
        ed.source());
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::make_iterator(gid_type const&)
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return iterator(this, m_domain, gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a const vertex iterator to the vertex with the
  /// specified descriptor.
  /// @param gid descriptor of the vertex to which the iterator will point.
  //////////////////////////////////////////////////////////////////////
  const_iterator make_iterator(gid_type const& gid) const
  {
    return const_iterator(this, m_domain, gid);
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
  /// @brief Creates an iterator to the specified gid, which is restricted
  /// to the provided domain.
  /// @param domain The domain to which the iterator will be restricted.
  /// @param gid The gid of the vertex to which the iterator will point.
  /// @return The iterator pointing to the specified gid, restricted to the
  /// given domain.
  //////////////////////////////////////////////////////////////////////
  const_iterator make_iterator(domain_type const& domain,
                                     gid_type const& gid) const
  {
    return const_iterator(this,domain,gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::make_reference(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  reference make_reference(index_type const& gid)
  {
    return reference(accessor_type(this, gid));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc directed_graph_base::make_reference(vertex_descriptor const&)
  //////////////////////////////////////////////////////////////////////
  const_reference make_reference(index_type const& gid) const
  {
    return const_reference(const_accessor_type(this, gid));
  }

  gid_type first(location_type loc) const
  {
    return m_domain.first(loc);
  }

  gid_type last(location_type loc) const
  {
    return m_domain.last(loc);
  }

  gid_type advance(gid_type const& gid, long long n) const
  {
    return m_domain.advance(gid,n);
  }

  size_t local_size(void) const
  {
    return m_domain.local_size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Advances the GID by the provided value, storing the result
  /// in a promise. This allows the advance to be non-blocking.
  /// This is needed by the distributed domain.
  //////////////////////////////////////////////////////////////////////
  void defer_advance(gid_type const& gid, long long n, promise<gid_type> p)
  {
    p.set_value(this->advance(gid, n));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Checks if the element specified by the GID is stored locally.
  /// The result is stored in a promise.
  /// This is needed by the distributed domain.
  //////////////////////////////////////////////////////////////////////
  void defer_contains(gid_type const& gid, promise<bool> p)
  {
    const bool res = this->container_manager().contains(gid);
    p.set_value(res);
  }

  size_t size(void) const
  {
    return domain().size();
  }

  domain_type const& domain(void) const
  {
    return m_domain;
  }


  typedef typename graph_distribution_traits<
    graph_distribution, container_type
  >::metadata_domain_type                           metadata_domain_type;

  typedef metadata_entry<
    metadata_domain_type, base_container_type*
  >                                                 dom_info_type;

  typedef std::vector<dom_info_type>                local_return_type;


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the metadata information for the base-containers on this
  /// location.
  /// @see metadata_entry.
  ////////////////////////////////////////////////////////////////////////
  local_return_type metadata(void)
  {
    local_return_type v;

    typename container_manager_type::iterator cit =
      this->container_manager().begin();
    typename container_manager_type::iterator cend =
      this->container_manager().end();

    for (; cit != cend; ++cit)
    {
      base_container_type& bc = *cit;

      if (!bc.domain().empty())
      {
        metadata_domain_type ndom(bc.domain(), this);

        v.push_back(dom_info_type(
          bc.cid(), ndom, &bc, LQ_CERTAIN, get_affinity(),
          this->get_rmi_handle(), this->get_location_id()
        ));
      }
    }

    return v;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the global number of base-containers in this distribution.
  ////////////////////////////////////////////////////////////////////////
  size_t num_base_containers(void)
  {
    return this->container_manager().m_ordering.m_total_num_bc;
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the metadata associated with the given @p gid by
  ///        setting the value of the promise.
  ///
  /// @param gid Id of the element of interest
  /// @param p Promise that will return the locality information to the
  ///          location that invoked the method.
  //////////////////////////////////////////////////////////////////////
  void defer_metadata_at(gid_type const& gid, promise<dom_info_type> p)
  {
    typedef typename container_manager_type::const_iterator c_iter_t;

    c_iter_t cit     = this->container_manager().begin();
    c_iter_t cit_end = this->container_manager().end();

    for (; cit != cit_end; ++cit)
    {
      base_container_type const& bc = *cit;
      if ((!bc.domain().empty()) && (bc.domain().contains(gid)))
      {
        p.set_value(dom_info_type(
          bc.cid(), metadata_domain_type(bc.domain(), this),
          const_cast<base_container_type*>(&bc),
          LQ_CERTAIN, get_affinity(),
          this->get_rmi_handle(), this->get_location_id()
        ));
        return;
      }
    }

    // The gid was not found.  Abort the execution.
    std::fprintf(stderr,
      "graph_distribution::defer_metadata_at: GID not on location"
      "specified by directory.\n");
    std::exit(1);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the metadata associated with the given @p index.
  ///
  /// @todo This method is currently used only by @ref static_array,
  ///       but the method should be generalized and added to
  ///       different distributions.
  ///
  /// @todo Propagate constness and add const qualifier to method.
  //////////////////////////////////////////////////////////////////////
  future<dom_info_type>
  metadata_at(gid_type const& gid)
  {
    typedef typename container_manager_type::const_iterator c_iter_t;

    c_iter_t cit     = this->container_manager().begin();
    c_iter_t cit_end = this->container_manager().end();

    for (; cit != cit_end; ++cit)
    {
      base_container_type const& bc = *cit;
      if ((!bc.domain().empty()) && (bc.domain().contains(gid)))
      {
        return make_ready_future(dom_info_type(
                 bc.cid(), metadata_domain_type(bc.domain(), this),
                 const_cast<base_container_type*>(&bc),
                 LQ_CERTAIN, get_affinity(),
                 this->get_rmi_handle(), this->get_location_id()));
      }
    }

    // Element was not found locally.  Retrieve the metadata from the location
    // at which the element is stored.
    typedef promise<dom_info_type> promise_type;
    promise_type p;
    auto f = p.get_future();
    this->directory().invoke_where(
      std::bind(
        [](p_object& d, gid_type const& gid, promise_type& p)
        {
          down_cast<graph_distribution&>(d).defer_metadata_at(
            gid, std::move(p));
        },
        std::placeholders::_1, std::placeholders::_2, std::move(p)), gid);

    return f;
  }
}; // struct graph_distribution

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_DISTRIBUTION_HPP
