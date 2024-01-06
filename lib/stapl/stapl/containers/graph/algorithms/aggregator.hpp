
/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_AGGREGATOR_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_AGGREGATOR_HPP

#include <algorithm>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

namespace stapl {

namespace aggr_algo_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to generate random numbers between [0, i).
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct rand_cont
{
private:
  boost::random::mt19937 m_rng;

public:
  rand_cont(unsigned int seed = get_location_id())
    : m_rng(seed)
  { }

  size_t operator()(size_t i)
  {
    return
      boost::random::uniform_int_distribution<location_type>(0, i-1)(m_rng);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Randomly shuffles location ids for all locations.
/// Used inside the aggregators to randomize communication order
/// in order to ease traffic.
/// @return The vector of shuffled location ids.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
inline std::vector<size_t>
init_shuffle_all_locs(location_type my_loc, size_t num_locations)
{
  std::vector<size_t> my_loc_ids;

  srand(my_loc);

  for (size_t i = 0; i < num_locations; ++i)
  {
    if (i != my_loc)
      my_loc_ids.push_back(i);
  }

  rand_cont rg;
  random_shuffle(my_loc_ids.begin(), my_loc_ids.end(), rg);

  return my_loc_ids;
}



//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to execute user-provided functor on the
/// target vertex. For use in @ref aggregator_apply.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct vertex_apply_wf
{
  template<typename Graph, typename UF>
  void operator() (Graph* g, UF const& uf) const
  {
    g->container_manager().apply(uf.target(), uf);
  }
};



//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to execute user-provided functor on the
/// target vertex's property.
/// @tparam F The type of the user-provided functor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename F>
struct vp_apply_aggr_helper
{
  F m_f;
  /// The target vertex on whose property the functor will be executed.
  size_t m_target;

  vp_apply_aggr_helper(void)
    : m_f(), m_target()
  { }

  vp_apply_aggr_helper(size_t const& target, F const& f)
    : m_f(f), m_target(target)
  { }

  size_t target(void) const
  { return m_target; }

  template<typename P>
  void operator() (P& p) const
  { m_f(p); }

  void define_type(typer& t)
  {
    t.member(m_f);
    t.member(m_target);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to wrap user-provided functor to provide a target.
/// @tparam F The type of the user-provided functor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename F>
struct target_aggr_helper
{
  F m_f;
  /// The target vertex on whom the functor will be executed.
  size_t m_target;

  target_aggr_helper(void)
    : m_f(), m_target()
  { }

  target_aggr_helper(size_t const& target, F const& f)
    : m_f(f), m_target(target)
  { }

  size_t target(void) const
  { return m_target; }

  template<typename P>
  void operator() (P& p) const
  { m_f(p); }

  void define_type(typer& t)
  {
    t.member(m_f);
    t.member(m_target);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to execute user-provided functor on the
/// graph.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct aggregator_helper_wf
{
  template<typename Graph, typename Elt>
  void operator() (Graph* g, Elt const& e) const
  { e(g); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to add provided edge to the graph.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct add_edge_wf
{
  template<typename Graph, typename Edge>
  void operator() (Graph* g, Edge const& e) const
  { g->add_edge_async(e); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to add provided edge with property to the graph.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct add_property_edge_wf
{
  template<typename Graph, typename PairEdgeProperty>
  void operator() (Graph* g, PairEdgeProperty const& p) const
  { g->add_edge_async(p.first, p.second); }
};

}  // aggr_algo_detail

//////////////////////////////////////////////////////////////////////
/// @brief Base class for aggregators.
///
/// Aggregates requests to vertices and flushes them out when either
/// a set number of requests have been aggregated (on a per-location basis)
/// or when the aggregator object is destroyed.
///
/// Performance optimization for hiding overhead of sending many small
/// messages over the runtime. Experiments have shown significant benefit
/// from using aggregators in algorithms requiring heavy communication
/// as is the case with the Graph500 benchmark.
///
/// @tparam Elem The type of the user-requests being aggregated.
/// @tparam Cont The type of the container (graphs).
/// @tparam Derived The most derived type for CRTP.
/// @todo Test performance of using std::unordered_map at scale,
/// swap std::vectors if performance is better (gForge Todo#985).
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Elem, typename Cont, typename Derived>
struct aggregator_base
{
private:
  typedef Derived                 derived_type;

protected:
  /// Container for storing requests to each location.
  std::vector<std::vector<Elem> > m_send_queue;
  /// @brief Container storing a random shuffling for all location-ids.
  /// Shuffling the location-ids provides better performance by
  /// easing network traffic.
  std::vector<size_t>             m_loc_ids;
  /// Pointer to the container where requests will be executed.
  Cont*                           m_pc;
  /// Maximum number of requests that will be aggregated.
  size_t                          m_max_msg_aggregate_sz;
  /// The ID of this location.
  size_t                          m_loc_id;

public:
  aggregator_base(Cont* pc, size_t max_msg_sz = 16384)
    : m_send_queue(pc->get_num_locations()),
      m_loc_ids(aggr_algo_detail::init_shuffle_all_locs(
        pc->get_location_id(), pc->get_num_locations()
      )),
      m_pc(pc), m_max_msg_aggregate_sz(max_msg_sz)
  { m_loc_id = pc->get_location_id(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the ID of the location where the specified element
  /// is stored.
  ///
  /// Derived classes may chose to overwrite this method to provide a
  /// suitable method to find what the element is. By default, the element
  /// GID is provided by the .target() method.
  //////////////////////////////////////////////////////////////////////
  size_t target_location(Elem const& x) const
  { return m_pc->locality(x.target()).location(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an element to the aggregator.
  ///
  /// This will trigger a flush if the queue for the target location has
  /// exceeded the maximum message size.
  //////////////////////////////////////////////////////////////////////
  void add(Elem const& x)
  {
    size_t loc = derived().target_location(x);
    m_send_queue[loc].push_back(x);
    if (m_send_queue[loc].size() >= m_max_msg_aggregate_sz) {
      // queue getting big -- send, flush, and clear queue...
      derived().flush(loc);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an element to the aggregator, when the target location
  /// is given.
  ///
  /// This will trigger a flush if the queue for the target location has
  /// exceeded the maximum message size.
  /// @param x The element to send.
  /// @param loc The location to send the element to.
  //////////////////////////////////////////////////////////////////////
  void add(Elem const& x, size_t loc)
  {
    m_send_queue[loc].push_back(x);
    if (m_send_queue[loc].size() >= m_max_msg_aggregate_sz) {
      // queue getting big -- send, flush, and clear queue...
      derived().flush(loc);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Destructor will also flush any remaining elements in the queues.
  //////////////////////////////////////////////////////////////////////
  ~aggregator_base()
  { manual_destruct(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Flush the queues for all locations.
  //////////////////////////////////////////////////////////////////////
  void flush(void)
  {
    this->manual_destruct();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Flush the queue for the provided location.
  /// @param i The location whose queue is to be flushed.
  //////////////////////////////////////////////////////////////////////
  void flush(size_t i)
  {
    m_pc->aggregate_vp_apply_async(i, m_send_queue[i]);
    m_send_queue[i].clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Flush the queue for this location. This is a local operation.
  //////////////////////////////////////////////////////////////////////
  void local_flush(void)
  {
    m_pc->distribution().aggregate_vp_apply_async(m_send_queue[m_loc_id]);
    m_send_queue[m_loc_id].clear();
  }

  Cont* container(void)
  { return m_pc; }

  void define_type(typer& t)
  {
    t.member(m_send_queue);
    t.member(m_loc_ids);
    t.member(m_pc);
    t.member(m_max_msg_aggregate_sz);
    t.member(m_loc_id);
  }

private:
  derived_type& derived(void)
  { return static_cast<derived_type&>(*this); }

  derived_type const& derived(void) const
  { return static_cast<derived_type const&>(*this); }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Flush all queues that are not empty.
  //////////////////////////////////////////////////////////////////////
  void manual_destruct(void)
  {
    // Flush out all rmi's...
    // Fence at the end needed as this is destroyed after the map-func.
    for (size_t i=0; i < m_loc_ids.size(); ++i) {
      if (m_send_queue[m_loc_ids[i]].size() != 0)
        derived().flush(m_loc_ids[i]);
    }

    derived().local_flush();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Aggregates requests to apply a functor (WF) on the element (Elem).
///
/// WF is provided with a pointer to the graph and an element.
/// @tparam Elem The type of the user-requests being aggregated.
/// @tparam Cont The type of the container (graphs).
/// @tparam WF The type of the functor to execute on  (graphs).
/// @tparam Derived The most derived type for CRTP.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Elem, typename Cont, typename WF, typename Derived>
struct aggregator_apply_impl
  : public aggregator_base<Elem, Cont, Derived>
{
  typedef aggregator_base<Elem, Cont, Derived> base_type;

  WF m_wf;

  aggregator_apply_impl(Cont* pc, size_t max_msg_sz = 16384)
    : base_type(pc, max_msg_sz)
  { }

  void flush(void)
  { this->manual_destruct(); }

  void flush(size_t i)
  {
    this->container()->aggregate_async(i, this->m_send_queue[i], m_wf);
    this->m_send_queue[i].clear();
  }

  void local_flush(void)
  {
    this->container()->distribution().aggregate_async(
      this->m_send_queue[this->m_loc_id],
      m_wf);
    this->m_send_queue[this->m_loc_id].clear();
  }

public:
  void define_type(typer& t)
  { t.member(m_wf); }
};



//////////////////////////////////////////////////////////////////////
/// @brief Applies Elem::operator() to vertex-property of Elem.target().
/// @tparam Elem The type of the user-requests being aggregated.
/// @tparam Cont The type of the container (graphs).
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Elem, typename Cont>
struct aggregator
  : public aggregator_base<Elem, Cont, aggregator<Elem, Cont> >
{
  typedef aggregator_base<Elem, Cont, aggregator<Elem, Cont> > base_type;

  aggregator(Cont* pc, size_t max_msg_sz = 16384)
    : base_type(pc, max_msg_sz)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Applies WF::operator() to a pointer to the graph on the
/// target location and an Elem.
/// @tparam Elem The type of the user-requests being aggregated.
/// @tparam Cont The type of the container (graphs).
/// @tparam WF The type of the functor to execute on (graphs).
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Elem, typename Cont,
         typename Wf = aggr_algo_detail::vertex_apply_wf>
struct aggregator_apply
  : public aggregator_apply_impl<Elem, Cont, Wf,
                                 aggregator_apply<Elem, Cont, Wf> >
{
  typedef aggregator_apply_impl<Elem, Cont, Wf,
                                aggregator_apply<Elem, Cont, Wf> > base_type;

  aggregator_apply(Cont* pc, size_t max_msg_sz = 16384)
    : base_type(pc, max_msg_sz)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Adds edges to the graph.
/// @tparam Cont The type of the container (graphs).
/// @tparam UseProperty Flag to indicate whether edge properties will
///         be taken into account.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Cont, bool UseProperty>
struct add_edge_aggregator
  : public aggregator_apply_impl<
      typename Cont::edge_descriptor,
      Cont,
      aggr_algo_detail::add_edge_wf,
      add_edge_aggregator<Cont, UseProperty>
    >
{
  using element_type = typename Cont::edge_descriptor;
  using base_type = aggregator_apply_impl<
    element_type,
    Cont,
    aggr_algo_detail::add_edge_wf,
    add_edge_aggregator<Cont, UseProperty>
  >;

  add_edge_aggregator(Cont* pc, size_t max_msg_sz = 16384)
    : base_type(pc, max_msg_sz)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Overwrites the base target_location() method to return
  /// the location of the source of the edge instead of its target.
  //////////////////////////////////////////////////////////////////////
  size_t target_location(element_type const& x)
  { return this->container()->locality(x.source()).location(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Adds edges with properties to the graph.
/// @tparam Cont The type of the container (graph).
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Cont>
struct add_edge_aggregator<Cont, true>
  : public aggregator_apply_impl<
      std::pair<typename Cont::edge_descriptor, typename Cont::edge_property>,
      Cont,
      aggr_algo_detail::add_property_edge_wf,
      add_edge_aggregator<Cont, true>
    >
{
  using element_type = std::pair<
    typename Cont::edge_descriptor, typename Cont::edge_property
  >;

  using base_type = aggregator_apply_impl<
    element_type,
    Cont,
    aggr_algo_detail::add_property_edge_wf,
    add_edge_aggregator<Cont, true>
  >;

  add_edge_aggregator(Cont* pc, size_t max_msg_sz = 16384)
    : base_type(pc, max_msg_sz)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Overwrites the base target_location() method to return
  /// the location of the source of the edge instead of its target.
  //////////////////////////////////////////////////////////////////////
  size_t target_location(element_type const& x)
  { return this->container()->locality(x.first.source()).location(); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Aggregates requests to apply a functor (Func) on elements of
/// a container.
///
/// Func is provided with an element.
///
/// @todo This aggregator should be based on the runtime object tunnel,
/// this should be updated to use said tunnel once it has been checked-in.
///
/// @tparam Container The type of the container (graphs).
/// @tparam Func The type of the functor to execute on elements.
/// @tparam Derived The most derived type for CRTP.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Container, typename Func>
class tunnel_aggregator
{
private:
  typedef typename Container::vertex_descriptor vertex_descriptor;

  // This should be uncommented and used once Ioannis commits the object tunnel.
#if 0
  typedef object_tunnel<Container, void, vertex_descriptor const&,
                        Func const&> tunnel_type;
  tunnel_type m_tunnel;
#endif
  Container* m_cont;

public:
  tunnel_aggregator(Container& cont)
    :
  // This should be uncommented and used once Ioannis commits the object tunnel.
#if 0
      m_tunnel(cont.get_rmi_handle(), &Container::template apply_set<Func>),
#endif
      m_cont(&cont)
  { }

  tunnel_aggregator(tunnel_aggregator const& other)
    :
  // This should be uncommented and used once Ioannis commits the object tunnel.
#if 0
      m_tunnel(other.m_cont->get_rmi_handle(),
               &Container::template apply_set<Func>),
#endif
      m_cont(other.m_cont)
  { }

  ~tunnel_aggregator()
  {
  // This should be uncommented and used once Ioannis commits the object tunnel.
#if 0
    m_tunnel.flush();
#endif
    rmi_fence();  // required to guarantee communication from aggr-flush().
  }

  void flush(void)
  {
  // This should be uncommented and used once Ioannis commits the object tunnel.
#if 0
    m_tunnel.flush();
#endif
    rmi_fence();  // required to guarantee communication from aggr-flush().
 }

  void add(vertex_descriptor const& gid, Func const& f)
  {
  // This should be uncommented and used once Ioannis commits the object tunnel.
#if 0
    m_tunnel.add(m_cont->locality(gid).location(), gid, f);
#endif
    m_cont->apply_set(gid, f);
  }

  void apply_set(vertex_descriptor const& gid, Func const& f)
  { add(gid, f); }

  void define_type(typer& t)
  { t.member(m_cont); }
};

} //namespace stapl

#endif
