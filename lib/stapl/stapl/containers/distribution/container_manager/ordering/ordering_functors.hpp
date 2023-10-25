/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_CM_ORDERING_ORDERING_FUNCTORS_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_CM_ORDERING_ORDERING_FUNCTORS_HPP

#include <stapl/containers/base/bc_base.hpp>
#include <stapl/utility/down_cast.hpp>

namespace stapl {

namespace ordering_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to advance forward the given @c gid, @c n
///        positions, returning the advanced gid using the given
///        promise.
///
/// This functor is used with the traversal_forward visitor.
//////////////////////////////////////////////////////////////////////
template <typename GID, typename BaseContainer>
class advance_fw
{
private:
  mutable size_t   m_n;
  mutable bool     m_first_call;
  GID              m_gid;
  promise<GID>     m_promise;
  mutable GID      m_ret_gid;

public:
  advance_fw(GID gid, long long n, promise<GID> const& promise)
    : m_n(std::abs(n)),
      m_first_call(true),
      m_gid(gid),
      m_promise(promise),
      m_ret_gid()
  { }

  bool operator()(bc_base* base_ptr, location_type, bool is_end)
  {
    BaseContainer* bc = down_cast<BaseContainer*>(base_ptr);

    if (is_end)
    {
      m_promise.set_value(m_ret_gid);
      return false;
    }

    if (bc == nullptr)
      return true;

    if (bc->size() == 0) {
      m_ret_gid = index_bounds<GID>::invalid();
      return true;
    }

    GID tmp_gid;
    if (m_first_call) {
      tmp_gid = m_gid;
      m_first_call = false;
    }
    else
      tmp_gid = bc->domain().first();

    size_t dist = bc->domain().distance(tmp_gid, bc->domain().last()) + 1;

    if (dist==m_n) {
      m_ret_gid = bc->domain().advance(tmp_gid, m_n);
    }

    if (dist>m_n) {
      m_ret_gid = bc->domain().advance(tmp_gid, m_n);
      m_promise.set_value(m_ret_gid);
      return false;
    }
    else {
      m_n = m_n - dist;
      return true;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_n);
    t.member(m_first_call);
    t.member(m_gid);
    t.member(m_promise);
    t.member(m_ret_gid);
  }
}; // class advance_fw


//////////////////////////////////////////////////////////////////////
/// @brief Functor used to advance backward the given @c gid, @c n
///        positions, returning the advanced gid using the given
///        promise.
///
/// This functor is used with the traversal_backward visitor.
//////////////////////////////////////////////////////////////////////
template <typename GID, typename BaseContainer>
class advance_bw
{
private:
  mutable size_t   m_n;
  mutable bool     m_first_call;
  GID              m_gid;
  promise<GID>     m_promise;
  mutable GID      m_ret_gid;

public:
  advance_bw(GID gid, long long n, promise<GID> const& promise)
    : m_n(std::abs(n)),
      m_first_call(true),
      m_gid(gid),
      m_promise(promise),
      m_ret_gid()
  { }

  bool operator()(bc_base* base_ptr, location_type, bool is_end)
  {
    BaseContainer* bc = down_cast<BaseContainer*>(base_ptr);

    if (is_end) {
      m_ret_gid = index_bounds<GID>::invalid();
      m_promise.set_value(m_ret_gid);
      return false;
    }
    if (bc==nullptr)
      return true;

    if (bc->size() == 0) {
      m_ret_gid = bc->domain().first();
      return true;
    }

    GID tmp_gid;
    if (m_first_call) {
      tmp_gid = m_gid;
      m_first_call = false;
    }
    else
      tmp_gid = bc->domain().last();

    size_t dist = bc->domain().distance(bc->domain().first(),tmp_gid) +1;

    if (dist>m_n) {
      m_ret_gid = bc->domain().advance(tmp_gid, -m_n);
      m_promise.set_value(m_ret_gid);
      return false;
    }
    else {
      m_n = m_n - dist;
      return true;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_n);
    t.member(m_first_call);
    t.member(m_gid);
    t.member(m_promise);
    t.member(m_ret_gid);
  }
}; // class advance_bw

template <typename Promise, typename Order, typename BaseContainer>
class find_first;

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to find the first gid, returning in the given
///        promise.
///
/// This functor is used with the traversal_forward visitor.
//////////////////////////////////////////////////////////////////////
template <typename Promise, typename Order, typename BaseContainer>
class find_fw
{
private:
  bool           m_first_call;
  Promise        m_promise;
  Order const*   m_order;

public:
  find_fw(Promise const& promise, Order const* order)
    : m_first_call(true),
      m_promise(promise),
      m_order(order)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the work function for forward traversal using the
  /// promise and ordering from the work function used in the find_first
  /// method of base container ordering.
  //////////////////////////////////////////////////////////////////////
  find_fw(find_first<Promise, Order, BaseContainer>& func)
    : m_first_call(true),
      m_promise(func.m_promise),
      m_order(func.m_order)
  { }

  bool operator()(bc_base* base_ptr, location_type, bool is_end)
  {
    BaseContainer* bc = down_cast<BaseContainer*>(base_ptr);

    typedef typename BaseContainer::gid_type gid_type;

    if (bc == nullptr && !m_first_call)
    {
      m_promise.set_value(index_bounds<gid_type>::invalid());
      return false;
    }

    m_first_call = false;

    BaseContainer* firstbc = bc;

    if (firstbc == nullptr)
      firstbc = down_cast<BaseContainer*>(m_order->first());

    if (firstbc != nullptr && firstbc->size() != 0)
    {
      m_promise.set_value(firstbc->domain().first());
      return false;
    }

    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_first_call);
    t.member(m_promise);
    t.member(m_order);
  }
}; // class find_fw

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to find the first gid, returning in the given
///        promise.
///
/// This functor is used with the find_first visitor.  It differs from
/// find_fw in that it traverses backward until the first base container
/// is found.  find_fw is then used to traverse forward over empty
/// base containers.
//////////////////////////////////////////////////////////////////////
template <typename Promise, typename Order, typename BaseContainer>
class find_first
{
private:
  Promise       m_promise;
  Order const*  m_order;

  // Allow find_bw to access members in order to avoid implementing getters.
  friend class find_fw<Promise, Order, BaseContainer>;

public:
  typedef Promise       promise_type;
  typedef BaseContainer base_container_type;

  find_first(Promise const& promise, Order const* order)
    : m_promise(promise),
      m_order(order)
  { }

  bool operator()(bc_base* base_ptr)
  {
    BaseContainer* bc = down_cast<BaseContainer*>(base_ptr);

    stapl_assert(bc != nullptr, "container conversion failed.");

#ifndef STAPL_NDEBUG
    // Ensure that the base container is indeed the first if we're on the
    // location the ordering reports to be the first.
    auto iter = m_order->m_follows.find(bc);

    stapl_assert(iter != m_order->m_follows.end(), "bc not found in follows");

    stapl_assert(iter->second.prev.location
                   == index_bounds<location_type>::invalid()
                 && iter->second.rank == 0,
                 "first bc has incorrect prev/rank");
#endif

    if (bc->size() != 0)
    {
      m_promise.set_value(bc->domain().first());
      return false;
    }

    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_promise);
    t.member(m_order);
  }
}; // class find_first


template <typename Promise, typename Order, typename BaseContainer>
class find_last;

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to find the last gid, returning in the given
///        promise.
///
/// This functor is used with the traversal_backward visitor.
//////////////////////////////////////////////////////////////////////
template <typename Promise, typename Order, typename BaseContainer>
class find_bw
{
private:
  bool             m_first_call;
  Promise          m_promise;
  Order const*     m_order;

public:
  find_bw(Promise const& promise, Order const* order)
    : m_first_call(true),
      m_promise(promise),
      m_order(order)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the work function for backward traversal using the
  /// promise and ordering from the work function used in the find_last
  /// method of base container ordering.
  //////////////////////////////////////////////////////////////////////
  find_bw(find_last<Promise, Order, BaseContainer>& func)
    : m_first_call(true),
      m_promise(func.m_promise),
      m_order(func.m_order)
  { }

  bool operator()(bc_base* base_ptr, location_type, bool is_end)
  {
    BaseContainer* bc = down_cast<BaseContainer*>(base_ptr);

    typedef typename BaseContainer::gid_type gid_type;

    if ((bc == nullptr) && (!m_first_call))
    {
      m_promise.set_value(index_bounds<gid_type>::invalid());
      return false;
    }

    m_first_call = false;

    BaseContainer* lastbc = bc;

    if (lastbc == nullptr)
      lastbc = down_cast<BaseContainer*>(m_order->last());

    if (lastbc != nullptr && lastbc->size() != 0)
    {
      m_promise.set_value(lastbc->domain().last());
      return false;
    }

    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_first_call);
    t.member(m_promise);
    t.member(m_order);
  }
}; // class find_bw


//////////////////////////////////////////////////////////////////////
/// @brief Functor used to find the last gid, returning in the given
///        promise.
///
/// This functor is used with the find_last method .  It differs from
/// find_bw in that it traverses forward until the last base container
/// is found.  find_bw is then used to traverse backward over empty
/// base containers.
//////////////////////////////////////////////////////////////////////
template <typename Promise, typename Order, typename BaseContainer>
class find_last
{
private:
  Promise       m_promise;
  Order const*  m_order;

  // Allow find_bw to access members in order to avoid implementing getters.
  friend class find_bw<Promise, Order, BaseContainer>;

public:
  typedef Promise       promise_type;
  typedef BaseContainer base_container_type;

  find_last(Promise const& promise, Order const* order)
    : m_promise(promise),
      m_order(order)
  { }

  bool operator()(bc_base* base_ptr)
  {
    BaseContainer* bc = down_cast<BaseContainer*>(base_ptr);

    stapl_assert(bc != nullptr, "container conversion failed.");

#ifndef STAPL_NDEBUG
    // Ensure that the base container is indeed the first if we're on the
    // location the ordering reports to be the first.
    auto iter = m_order->m_follows.find(bc);

    stapl_assert(iter != m_order->m_follows.end(), "bc not found in find_last");

    stapl_assert(iter->second.next.location
                   == index_bounds<location_type>::invalid(),
                 "first bc has incorrect next");
#endif

    if (bc->size() != 0)
    {
      m_promise.set_value(bc->domain().last());
      return false;
    }

    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_promise);
    t.member(m_order);
  }
}; // class find_last


//////////////////////////////////////////////////////////////////////
/// @brief Functor used to compute the distance between to given gids,
///        returning the computed distance in the given promise.
///
/// This functor is used with the traversal_forward visitor.
//////////////////////////////////////////////////////////////////////
template <typename GID, typename BaseContainer>
class distance_fw
{
private:
  mutable size_t      m_size;
  mutable bool        m_first_call;
  GID                 m_first;
  GID                 m_last;
  promise<size_t>     m_promise;

public:
  distance_fw(GID const& gida, GID const& gidb, promise<size_t> const& promise)
    : m_size(0),
      m_first_call(true),
      m_first(gida),
      m_last(gidb),
      m_promise(promise)
  { }

  bool operator()(bc_base* base_ptr, location_type location, bool is_end)
  {
    BaseContainer* bc = down_cast<BaseContainer*>(base_ptr);

    if (is_end)
    {
      m_promise.set_value(m_size);
      return false;
    }

    if (bc == nullptr)
      return true;

    GID tmp_gid;

    if (m_first_call)
    {
      tmp_gid      = m_first;
      m_first_call = false;
    }
    else
      tmp_gid = bc->domain().first();

    size_t dist;

    if (m_last.base_container_equal(bc, location))
    {
      dist    = bc->domain().distance(tmp_gid, m_last) +1;
      m_size += dist;
      m_promise.set_value(m_size);
      return false;
    }

    if (m_first.base_container_equal(bc, location))
      dist = bc->domain().distance(tmp_gid, bc->domain().last()) + 1;
    else
      dist = bc->size();

    m_size += dist;
    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_first_call);
    t.member(m_last);
    t.member(m_promise);
  }
}; // class distance_fw


//////////////////////////////////////////////////////////////////////
/// @brief Functor used to compute the distance in associative
///        containers between to given gids, returning the computed
///        distance in the given promise.
///
/// This functor is used with the traversal_forward visitor.
//////////////////////////////////////////////////////////////////////
template <typename GID, typename BaseContainer>
class distance_fw_associative
{
private:
  mutable size_t      m_size;
  mutable bool        m_first_call;
  GID                 m_first;
  GID                 m_last;
  promise<size_t>     m_promise;

public:
  distance_fw_associative(GID const& gida, GID const& gidb,
                          promise<size_t> const& promise)
    : m_size(0),
      m_first_call(true),
      m_first(gida),
      m_last(gidb),
      m_promise(promise)
  { }

  bool operator()(bc_base* base_ptr, location_type, bool is_end)
  {
    BaseContainer* bc = down_cast<BaseContainer*>(base_ptr);

    if (is_end)
    {
      m_promise.set_value(m_size);
      return false;
    }

    if (bc == nullptr)
      return true;

    GID tmp_gid;
    if (m_first_call) {
      tmp_gid = m_first;
      m_first_call = false;
    }
    else
      tmp_gid = bc->domain().first();

    size_t dist;
    if (bc->domain().contains(m_last)) {
      dist = bc->domain().distance(tmp_gid,m_last);
      m_size += dist;
      m_promise.set_value(m_size);
      return false;
    }

    if (bc->domain().contains(m_first))
    {
      dist = bc->domain().distance(tmp_gid,bc->domain().last())+1;
    }
    else
    {
      dist = bc->size();
    }

    m_size += dist;
    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_first_call);
    t.member(m_last);
    t.member(m_promise);
  }
}; // class distance_fw_associative


//////////////////////////////////////////////////////////////////////
/// @brief Functor used to search in a stapl::list for a specific gid
///        in the given range [first..last], returning if the gid was
///        found or not using the given promise.
///
/// This functor is used with the traversal_forward visitor.
/// @tparam GID Type of the list's gid.
//////////////////////////////////////////////////////////////////////
template <typename GID, typename BaseContainer>
class search_fw
{
private:
  bool              m_first_call;
  GID               m_first;
  GID               m_last;
  GID               m_gid;
  promise<bool>     m_promise;

public:
  search_fw(GID const& first,
            GID const& last,
            GID const& gid,
            promise<bool> const& promise)
    : m_first_call(true),
      m_first(first),
      m_last(last),
      m_gid(gid),
      m_promise(promise)
  { }

  bool operator()(bc_base* base_ptr, location_type location, bool is_end)
  {
    BaseContainer* bc = down_cast<BaseContainer*>(base_ptr);

    if (is_end)
    {
      m_promise.set_value(false);
      return false;
    }

    if (bc == nullptr)
      return true;

    bool last_reached = false;

    GID tmp_first;

    if (m_first.base_container_equal(bc, location))
      tmp_first = m_first;
    else
      tmp_first = bc->domain().first();

    GID tmp_last;

    if (m_last.base_container_equal(bc, location))
    {
      tmp_last     = m_last;
      last_reached = true;
    }
    else
      tmp_last = bc->domain().last();

    if (m_first_call)
    {
      m_first_call = false;

      if (m_first.base_container_equal(bc, location)
          && m_gid.base_container_equal(m_first))
      {
        m_promise.set_value(bc->search(tmp_first, tmp_last, m_gid));
        return false;
      }
    }
    else
    {
      if (m_last.base_container_equal(bc, location)
          && m_gid.base_container_equal(m_last))
      {
        m_promise.set_value(bc->search(tmp_first, tmp_last, m_gid));
        return false;
      }

      if (m_gid.base_container_equal(bc, location))
      {
        m_promise.set_value(true);
        return false;
      }
    }

    if (last_reached)
    {
      m_promise.set_value(false);
      return false;
    }

    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_first_call);
    t.member(m_first);
    t.member(m_last);
    t.member(m_gid);
    t.member(m_promise);
  }
}; // class search_fw

} // namespace ordering_detail

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_CM_ORDERING_ORDERING_FUNCTORS_HPP
