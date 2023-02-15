/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_LIST_DISTRIBUTED_DOMAIN_HPP
#define STAPL_DOMAINS_LIST_DISTRIBUTED_DOMAIN_HPP

#include <cstddef>
#include <functional>
#include <iosfwd>

#include <stapl/containers/type_traits/distribution_traits.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which is identity function parameter unless it is a
///   @ref future, in which case it blocks.
/// @tparam T Return object type.
/// Used to generalize value returning code with asynchronous,
///   promise setting implementations.
//////////////////////////////////////////////////////////////////////
template<typename T>
class value_returner
{
public:
  typedef promise<T> promise_type;

private:
  future<T> m_future;

public:
  promise_type get_promise(void)
  {
    promise_type p;
    m_future = p.get_future();
    return p;
  }

  T operator()(T const& t) const
  {
    return t;
  }

  T operator()(void)
  {
    return m_future.get(); // sync_rmi equivalent
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor which sets a stored promise with the passed parameter
///   unless the parameter is @ref future, in which case it is a noop.
/// @tparam T Return object type.
/// Used to generalize asynchronous, promise setting implementations
///   with value returning code.
//////////////////////////////////////////////////////////////////////
template<typename T>
class void_returner
{
public:
  typedef promise<T> promise_type;

private:
  promise_type m_promise;

public:
  explicit void_returner(promise_type p)
    : m_promise(std::move(p))
  { }

  promise_type& get_promise(void)
  {
    return m_promise;
  }

  void operator()(T const& t)
  {
    m_promise.set_value(t);
  }

  void operator()(void) const
  { }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Defines a distributed domain using the given @c
///        Distribution to support the domain's methods.
///
/// This domain is specialized to work for the pList.
//////////////////////////////////////////////////////////////////////
template <typename Distribution>
class list_distributed_domain
{
public:
  typedef typename distribution_traits<Distribution>::gid_type gid_type;
  typedef gid_type                                             index_type;
  typedef std::size_t                                          size_type;

private:
  p_object_pointer_wrapper<const Distribution> m_dist;
  index_type                                   m_first;
  index_type                                   m_last;

  //////////////////////////////////////////////////////////////////////
  /// @brief Propagate constness and remove const_cast.
  //////////////////////////////////////////////////////////////////////
  Distribution /*const*/& distribution(void) const
  {
    return const_cast<Distribution&>(*m_dist);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Shared implementation of @ref defer_contains and contains.
  /// @tparam Returner Functor that receives return value (or a promise)
  ///   and either returns the value (blocking if necessary) or sets promise
  ///   if possible (for asynchronous clients).
  //////////////////////////////////////////////////////////////////////
  template<typename Returner>
  auto contains_impl(index_type const& g, Returner&& returner) const
    ->decltype(returner(true))
  {
    typedef promise<bool> promise_type;

    if (!m_dist)
      return returner(false);

    if (m_first.valid() && m_last.valid())
    {
      if (m_first.base_container_equal(m_last)
          && m_first.m_location != g.m_location)
        return returner(false);

      auto&& p = returner.get_promise();

      if (m_first.m_location == distribution().get_location_id())
      {
        distribution().container_manager().defer_search(
          m_first, m_last, g, std::move(p));

        return returner();
      }

      // else
      distribution().directory().invoke_where(
        std::bind(
          [](p_object& d, list_distributed_domain const& dom,
             index_type const& g, promise_type& p)
          {
            down_cast<Distribution&>(d).contains_helper(dom, g, std::move(p));
          },
          std::placeholders::_1, *this, g, std::move(p)),
        m_first);

      return returner();
    }

    auto&& p = returner.get_promise();

    distribution().directory().invoke_where(
      std::bind(
        [](p_object& d, index_type const& g, promise_type& p)
          { down_cast<Distribution&>(d).defer_contains(g, std::move(p)); },
        std::placeholders::_1, std::placeholders::_2, std::move(p)),
      g);

    return returner();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Shared implementation of @ref defer_advance and advance.
  /// @tparam Returner Functor that receives return value (or a promise)
  ///   and either returns the value (blocking if necessary) or sets promise
  ///   if possible (for asynchronous clients).
  //////////////////////////////////////////////////////////////////////
  template<typename Returner>
  auto advance_impl(index_type const& g,
                    long long n,
                    bool globally,
                    Returner&& returner) const
    ->decltype(returner(g))
  {
    typedef promise<gid_type> promise_type;

    if (!g.valid())
      return returner(g);

    if (m_first.valid() && m_last.valid()
      && m_first.base_container_equal(m_last))
      globally = false;

    if (g.m_location == distribution().get_location_id())
    {
      if (g.m_base_container->end() == g.m_pointer && n == -1)
        return returner(last());

      auto&& p = returner.get_promise();

      distribution().container_manager().defer_advance(
        g, n, globally, std::move(p));

      return returner();
    }

    // else
    auto&& p = returner.get_promise();

    distribution().directory().invoke_where(
      std::bind(
        [](p_object& d, index_type const& g, long long n,
           bool globally, promise_type& p)
        {
          down_cast<Distribution&>(d).defer_advance(
            g, n, globally, std::move(p));
        },
        std::placeholders::_1, std::placeholders::_2,
        n, globally, std::move(p)),
      g);

    return returner();
  }

public:
  list_distributed_domain(void)
    : m_first(),
      m_last()
  { }

  explicit list_distributed_domain(Distribution const& dist)
    : m_dist(&dist),
      m_first(),
      m_last()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new domain by restricting the given domain
  ///        (@p dom) to be [first..last].
  //////////////////////////////////////////////////////////////////////
  list_distributed_domain(index_type const& first,
                          index_type const& last,
                          list_distributed_domain const& dom)
    : m_dist(dom.m_dist),
      m_first(first),
      m_last(last)
  { }

  list_distributed_domain(index_type const& first,
                          index_type const& last,
                          Distribution const& dist)
    : m_dist(&dist),
      m_first(first),
      m_last(last)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the first gid in the domain.
  ///
  /// The method uses a visitor pattern to find the first gid.
  /// starting from the given location (@p loc).
  //////////////////////////////////////////////////////////////////////
  index_type first(void) const
  {
    if (m_first.valid())
      return m_first;

    promise<gid_type> p;
    auto f = p.get_future();

    distribution().container_manager().find_first(std::move(p));

    return f.get(); // sync_rmi() equivalent
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the last valid gid in the domain.
  ///
  /// The method uses a visitor pattern to find the last valid gid.
  //////////////////////////////////////////////////////////////////////
  index_type last(void) const
  {
    if (m_last.valid())
      return m_last;

    promise<gid_type> p;
    auto f = p.get_future();

    distribution().container_manager().find_last(std::move(p));

    return f.get(); // sync_rmi() equivalent
  }

  index_type open_last(location_type = invalid_location_id) const
  {
    if (m_last.valid())
      return advance(last(), 1, false);
    return advance(last(), 1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets @ref promise @p p with the gid resulting from advancing
  ///    the given gid @p g, @p n positions.
  ///
  /// To determine the correct end of the container, the parameter @p
  /// globally indicates if the traversal is at the global level (@c
  /// True) or at the local level (@c False).
  ///
  /// @todo @p n is supposed to be a positive integral, yet it is tested against
  ///       -1.
  //////////////////////////////////////////////////////////////////////
  void defer_advance(index_type const& g,
                     long long n,
                     bool globally, promise<gid_type> p) const
  {
    advance_impl(g, n, globally, detail::void_returner<gid_type>{p});
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the gid resulting of advance the given gid  @p g,
  ///        @p n positions.
  ///
  /// To determine the correct end of the container, the parameter @p
  /// globally indicates if the traversal is at the global level (@c
  /// True) or at the local level (@c False).
  ///
  /// @todo @p n is supposed to be a positive integral, yet it is tested against
  ///       -1.
  //////////////////////////////////////////////////////////////////////
  index_type
  advance(index_type const& g, long long n, bool globally = true) const
  {
    return advance_impl(g, n, globally, detail::value_returner<gid_type>{});
  }

  size_type size(void) const
  {
    if (m_first.valid() && m_last.valid())
    {
      typedef promise<size_type> promise_type;

      promise_type p;
      auto f = p.get_future();

      if (m_first.m_location == distribution().get_location_id()) {
        distribution().container_manager().defer_distance(
          m_first, m_last, std::move(p));
      } else {
        distribution().directory().invoke_where(
          std::bind(
            [](p_object& d, index_type const& first, index_type const& last,
               promise_type& p)
            {
              down_cast<Distribution&>(d).defer_distance(
                first, last, std::move(p));
            },
            std::placeholders::_1, std::placeholders::_2, m_last, std::move(p)),
          m_first);
      }

      return f.get(); // sync_rmi() equivalent
    }

    if (!m_dist)
      return 0;

    if (distribution().get_num_locations() == 1)
      return local_size();

    return sync_reduce_rmi(std::plus<size_type>(),
                           distribution().get_rmi_handle(),
                           &Distribution::local_size);
  }

  size_type local_size(void) const
  {
    return distribution().container_manager().size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets @ref boolean promise @p p based on whether @p g exists
  ///   in this domain.
  //////////////////////////////////////////////////////////////////////
  void defer_contains(index_type const& g, promise<bool> p) const
  {
    contains_impl(g, detail::void_returner<bool>{p});
  }

  bool contains(index_type const& g) const
  {
    return contains_impl(g, detail::value_returner<bool>{});
  }

  bool empty(void) const
  {
    return size() == 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns how many positions are needed to advance from the
  ///        gid @p a to reach the gid @p b.
  /// @note The returned value does not define in which direction
  ///       advance.
  /// @todo Change to use traversal_forward (visitor pattern).
  //////////////////////////////////////////////////////////////////////
  size_type distance(index_type const& a, index_type const& b) const
  {
    auto dist = std::distance(distribution().find(a), distribution().find(b));
    return (dist < 0) ? -dist : dist;
  }

  bool is_same_container_domain(void) const
  {
    return !m_first.valid() && !m_last.valid();
  }

  bool is_contiguous(void) const
  {
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the resulting intersection with the given @p other
  ///        domain.
  //////////////////////////////////////////////////////////////////////
  list_distributed_domain operator&(list_distributed_domain const& other) const
  {
    if (empty() || other.empty())
      return list_distributed_domain();

    index_type olower = other.first();
    index_type oupper = other.last();

    if (m_first.valid() && m_last.valid())
      if (m_first.base_container_equal(m_last))
        if (m_first.m_location != olower.m_location
            && m_last.m_location != oupper.m_location)
          return list_distributed_domain();

    const bool contains_first = other.contains(first());
    const bool contains_last  = other.contains(last());

    if (contains_first && contains_last)
      return *this;

    if (!contains_first && !contains_last) {
      if (contains(other.first()))
        return other;
      else
        return list_distributed_domain();
    }
    else {
      if (contains_first)
        olower = first();

      if (contains_last) {
        oupper = last();
      }
    }
    return list_distributed_domain(olower, oupper, *this);
  }

  void define_type(typer& t)
  {
    t.member(m_dist);
    t.member(m_first);
    t.member(m_last);
  }
}; // class list_distributed_domain


template<typename D>
std::ostream& operator<<(std::ostream& os, list_distributed_domain<D> const& d)
{
  if (d.empty())
    return os << "empty";

  // else
  return os << "[" << d.first() << ".." << d.last() << "]";
}

} // namespace stapl

#endif // STAPL_DOMAINS_LIST_DISTRIBUTED_DOMAIN_HPP
