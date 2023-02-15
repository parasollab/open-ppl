/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_DISTRIBUTION_BASE_HPP
#define STAPL_CONTAINERS_GRAPH_DISTRIBUTION_BASE_HPP

#include <boost/bind/bind.hpp>
#include <stapl/containers/graph/functional.hpp>
#include <stapl/containers/distribution/distribution.hpp>
#include <stapl/containers/distribution/operations/base.hpp>
#include <stapl/containers/graph/distribution_traits.hpp>
#include <stapl/containers/distribution/graph_metadata.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor that is invoked on the source vertex location
///        to set the promise to the correct value for has_edge.
///
/// @tparam Derived Type of the graph_distribution
/// @tparam VD Vertex descriptor type
//////////////////////////////////////////////////////////////////////
template<typename Derived, typename VD>
class has_edge_helper
{
private:
  stapl::promise<bool> m_promise;
  VD m_target;

  using base_container_type =
    typename distribution_traits<Derived>::base_container_type;

public:
  has_edge_helper(stapl::promise<bool> p, VD const& target)
    : m_promise(std::move(p)), m_target(target)
  { }

  void operator()(p_object& d, VD const& source)
  {
    auto opt = down_cast<Derived&>(d).container_manager()
      .contains_invoke(
        source, &base_container_type::has_edge, source, m_target
      );

    stapl_assert(opt, "Source vertex not found");

    m_promise.set_value(*opt);
  }

  void define_type(typer& t)
  {
    t.member(m_promise);
    t.member(m_target);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Base distribution class for the graph containers.
/// @ingroup pgraphDist
///
/// @tparam Container Type of the container that is managing
/// this distribution.
/// @tparam Derived The most derived distribution class
////////////////////////////////////////////////////////////////////////
template<typename Container, typename Derived>
class graph_distribution_base
  : public distribution<Container>,
    public operations::base<Derived>
{
  using base_type = distribution<Container>;
  using derived_type = Derived;

public:
  using vertex_descriptor =
    typename distribution_traits<derived_type>::gid_type;
  using base_container_type =
    typename distribution_traits<derived_type>::base_container_type;

  template<typename... Args>
  graph_distribution_base(Args&&... args)
    : base_type(std::forward<Args>(args)...)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determines whether there exists an edge between two vertices
  /// @return A future representing the answer to this query
  //////////////////////////////////////////////////////////////////////
  future<bool> has_edge(vertex_descriptor const& source,
                        vertex_descriptor const& target)
  {
    if (auto opt = this->container_manager().contains_invoke(source,
          &base_container_type::has_edge, source, target)) {
      return make_ready_future(*opt);
    }

    promise<bool> p;
    auto f = p.get_future();

    this->directory().invoke_where(
      detail::has_edge_helper<Derived, vertex_descriptor>(
        std::move(p), target
      ), source
    );

    return f;
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
    for (typename Cont::const_iterator it = cont.begin();
         it != cont.end();
         ++it)
      this->container_manager().apply(it->target(), f);
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
        (void (graph_distribution_base::*)(Cont const&, F const&))
          & graph_distribution_base::aggregate_apply_async,
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
        (void (graph_distribution_base::*)(size_t, Cont const&, F const&))
          & graph_distribution_base::guarded_aggregate_apply_async,
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
        (void (graph_distribution_base::*)(Cont const&, F const&))
          & graph_distribution_base::aggregate_apply_async,
        cont,
        f);

    this->unlock();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a functor to the element at a given GID in a non-concurrent
  ///        manner. That is, if the graph is already in an apply_set, then
  ///        this apply_set will be deferred until the one in progress has
  ///        ended.
  ///
  /// @param gid The GID of the vertex
  /// @param f The functor to apply
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void guarded_apply_set(vertex_descriptor const& gid, F&& f)
  {
    if (!this->try_lock()) {
      unordered::async_rmi(
        this->get_location_id(),
        this->get_rmi_handle(),
        (void (graph_distribution_base::*)(vertex_descriptor const&, F&&))
          & graph_distribution_base::guarded_apply_set,
        gid,
        std::forward<F>(f));

      return;
    }

    this->unordered_apply_set(gid, std::forward<F>(f));

    this->unlock();
  }
};

} // namespace detail

} // namespace stapl

#endif
