/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_EDGE_UTILITY_HPP
#define STAPL_PARAGRAPH_EDGE_UTILITY_HPP

#include <stapl/algorithms/functional.hpp>

namespace stapl {

class edge_container;

// Place holder num succs value used to indicate unknown number of successors.
const size_t defer_spec = std::numeric_limits<int>::max();


//////////////////////////////////////////////////////////////////////
/// @brief Custom PARAGRAPH edge storage a proxy.
/// @tparam Proxy The proxy type to be stored in @ref edge_container.
/// @ingroup pgEdgeContainer
/// Define default construction and assignment semantics for use
/// by the edge_container.
///
/// @sa df_stored_type
//////////////////////////////////////////////////////////////////////
template<typename Proxy>
struct proxy_holder
{
  typedef Proxy proxy_type;

  /// @brief The underlying proxy passed on the edge.
  Proxy m_proxy;

  //////////////////////////////////////////////////////////////////////
  /// @brief Default constructor initializes proxy as null reference.
  //////////////////////////////////////////////////////////////////////
  proxy_holder()
   : m_proxy(null_reference())
  { }

  proxy_holder(Proxy const& proxy)
    : m_proxy(proxy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Conversion operator to underlying proxy type.
  //////////////////////////////////////////////////////////////////////
  operator Proxy() const
  {
    return m_proxy;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Assignment operator redirects to @p reset method of
  /// @p proxy_core_access.
  ///
  /// @sa proxy_core_access::reset
  //////////////////////////////////////////////////////////////////////
  proxy_holder& operator=(proxy_holder const& rhs)
  {
    if (this != &rhs)
      proxy_core_access::reset(this->m_proxy, rhs.m_proxy);

    return *this;
  }

  void define_type(typer& t)
  {
    // avoid calling define_type on proxy, instead on its base
    // proxy purposely doesn't define &
    t.member(proxy_core_access::accessor(m_proxy));
  }
}; //class proxy_holder


//////////////////////////////////////////////////////////////////////
/// @brief Custom PARAGRAPH edge storage for pairs of proxies.
/// @tparam Proxy1 First proxy type of pair.
/// @tparam Proxy2 Second proxy type of pair.
/// @ingroup pgEdgeContainer
///
/// Define default construction and assignment semantics for use
/// by the edge_container.
///
/// @sa df_stored_type
//////////////////////////////////////////////////////////////////////
template<typename Proxy1, typename Proxy2>
struct proxy_pair_holder
{
  typedef std::pair<Proxy1,Proxy2> pair_t;

  /// @brief The underlying pair of proxies passed on the edge.
  pair_t m_pair;

  //////////////////////////////////////////////////////////////////////
  /// @brief Default constructor initialized both proxies as null references.
  //////////////////////////////////////////////////////////////////////
  proxy_pair_holder()
   : m_pair(Proxy1(null_reference()), Proxy2(null_reference()))
  { }

  proxy_pair_holder(pair_t const& value)
   : m_pair(value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Conversion operator to underlying pair type.
  //////////////////////////////////////////////////////////////////////
  operator pair_t() const
  {
    return m_pair;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Assignment operator redirects to @p reset method of
  /// @p proxy_core_access for each proxy.
  ///
  /// @sa proxy_core_access::reset
  //////////////////////////////////////////////////////////////////////
  proxy_pair_holder& operator=(proxy_pair_holder const& rhs)
  {
    if (this != &rhs)
    {
      proxy_core_access::reset(this->m_pair.first, rhs.m_pair.first);
      proxy_core_access::reset(this->m_pair.second, rhs.m_pair.second);
    }

    return *this;
  }

  void define_type(typer& t)
  {
    t.member(m_pair);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction computing the actual type that will be stored
/// for a requested edge type.
/// @ingroup pgEdgeContainer
///
/// Internally, it is useful to modify the type for some special cases outlined
/// below in the various partial specializations of the class.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct df_stored_type
{
  typedef T type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for void edge types (i.e., signals).
/// @ingroup pgEdgeContainer
///
/// Internally pass an int to simplify code paths that expect to receive a
/// parameter (can't pass void).
//////////////////////////////////////////////////////////////////////
template<>
struct df_stored_type<void>
{
  typedef int type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for proxy edge types.
/// @ingroup pgEdgeContainer
///
/// Proxies' assignment operators are overloaded to behave like a real reference
/// (i.e., redirect to reference object).  We use a @ref proxy_holder to give us
/// a proper assignment operator to use.
///
/// @sa proxy_holder
//////////////////////////////////////////////////////////////////////
template<typename T, typename A>
struct df_stored_type<proxy<T, A>>
{
  typedef proxy_holder<proxy<T, A>> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for pairs of proxies.
/// @ingroup pgEdgeContainer
///
/// Some algorithms use pairs of proxies to denote ranges and transmit these
/// through the edge container.  Use a @ref proxy_pair_holder for the same
/// reason discussed in simple proxy specialization of @ref df_stored_type.
///
/// @sa proxy_pair_holder
//////////////////////////////////////////////////////////////////////
template<typename T1, typename T2, typename A1, typename A2>
struct df_stored_type<std::pair<proxy<T1,A1>, proxy<T2,A2>>>
{
  typedef proxy_pair_holder<proxy<T1,A1>, proxy<T2,A2>> type;
};


// What's the state of the requested flow for this tid on this location?
// Have I:
//   (0) I've have requested nothing
//   (1) Just requested a SIGNAL that the task is done.
//   (2) Requested a filtered version of the retval,
//   (3) requested the entire retval
//
enum edge_request_type
  { NONE = -1, SIGNAL = 0, FILTERED = 1, FULL = 2, REMOTE = 3};


// Signals edge_container's flow setup methods whether or not a remote notifier
// needs to be sent to the producer location when adding a new local consumer
// on a location.
enum df_add_status { REMOTE_REQ_COVERED, SEND_NOTIFIER_REQ };


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Filtering function object associated internally in the
///   @p edge_container implementation for edges representing only a SIGNAL
///   (i.e., an explicit predecessor passed to @p paragraph_view::add_task.
/// @ingroup pgEdgeContainer
///
/// To unify the implementation, the function operand returns full value
/// passed to it, but this is not made available to the predecessor task.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct df_null_functor
  : public ro_unary_function<T, T>
{
  template<typename Ref1>
  T operator()(Ref1 x)
  {
    return x;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Need by Boost.Function for comparison.  Since there are no data
  /// members, any two objects of this type are equivalent.
  //////////////////////////////////////////////////////////////////////
  bool operator==(df_null_functor const&) const
  {
    return true;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Filtering function object associated internally in the
///   @p edge_container implementation for edges representing FULL consumption
///   a producer task's value by the successor.  Hence it is an identity filter,
///   returning the full value passed to it.
/// @ingroup pgEdgeContainer
///
/// @todo Determine if nonconst function operator signature is still needed.
/// @todo Consider consolidating with other identify functors in stapl.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct df_identity
  : public ro_unary_function<T, T>
{
  template<typename Ref1>
  T operator()(Ref1 x)
  {
    return x;
  }

  template<typename Ref1>
  T operator()(Ref1 x) const
  {
    return x;
  }

  T&& operator()(T&& val) const
  {
    return std::move(val);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Need by Boost.Function for comparison.  Since there are no data
  /// members, any two objects of this type are equivalent.
  //////////////////////////////////////////////////////////////////////
  bool operator==(df_identity const&) const
  {
    return true;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Template metafunction reflecting edge_request_type value
///   associated with the specified filter type.
/// @ingroup pgEdgeContainer
///
/// @tparam Functor The filter type passed to an @p edge_entry method.
///
/// The primary template matches all user specified templates (originally
/// passed to @p consume).  Return FILTERED request type.
///
/// @sa consume
/// @sa edge_entry::add_local_notifier
//////////////////////////////////////////////////////////////////////
template<typename Functor>
struct compute_edge_request_type
{
  static const edge_request_type value = FILTERED;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for df_null_functor<T>.  Reflect SIGNAL
///   request type.
/// @ingroup pgEdgeContainer
//////////////////////////////////////////////////////////////////////
template<typename T>
struct compute_edge_request_type<df_null_functor<T>>
{
  static const edge_request_type value = SIGNAL;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for df_identity<T>.  Reflect FULL
///   request type.
/// @ingroup pgEdgeContainer
//////////////////////////////////////////////////////////////////////
template<typename T>
struct compute_edge_request_type<df_identity<T>>
{
  static const edge_request_type value = FULL;
};


//////////////////////////////////////////////////////////////////////
/// @brief Unary functor to compare the return value of any object's
///   @p request method with a given edge_request_type variable.
/// @ingroup pgEdgeContainer
///
/// Used to search versions list in @p edge_entry.
///
/// @sa edge_entry::iterator_lookup_version
//////////////////////////////////////////////////////////////////////
struct compare_req
{
  /// request to compare against
  edge_request_type m_req;

  compare_req(edge_request_type req)
    : m_req(req)
   { }

  template<typename T>
  bool operator()(T const& other) const
  {
    return other.request() == m_req;
  }
};

} // namesapce detail

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_EDGE_UTILITY_HPP

