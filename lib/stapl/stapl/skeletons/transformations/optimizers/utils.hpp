/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_UTILS_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_UTILS_HPP

#include <type_traits>
#include <stapl/views/type_traits/has_iterator.hpp>
#include <stapl/views/type_traits/is_proxy.hpp>
#include <stapl/views/type_traits/is_view.hpp>
#include <stapl/utility/tuple/apply.hpp>

namespace stapl {
namespace skeletons {
namespace optimizers {
namespace helpers {

template<typename View, typename... Tail>
struct dimensions_of_first_view
{
  using type = typename dimension_traits<typename std::decay<View>::type>::type;
};

//////////////////////////////////////////////////////////////////////
/// @brief A dummy function used to enable prefix increments on the
/// view id set iterators.
/// @internal
//////////////////////////////////////////////////////////////////////
template<typename ...Iter>
void no_op(Iter&&...)
{ }

//////////////////////////////////////////////////////////////////////
/// @brief Advances a given GID by one based on the traversal of a domain.
/// @internal
//////////////////////////////////////////////////////////////////////
template<typename Dom, typename GID>
GID& advance_domain(Dom&& d, GID& gid)
{
  gid = d.advance(gid, 1);
  return gid;
}

//////////////////////////////////////////////////////////////////////
/// @brief Returns the size of the first view's domain in a pack of
///        views.
/// @internal
//////////////////////////////////////////////////////////////////////
template<typename V0, typename... V>
std::size_t view_size(V0&& v0, V... v)
{
  return v0.domain().size();
}

//////////////////////////////////////////////////////////////////////
/// @brief Returns a reference to the view element at given GID.
/// @internal
//////////////////////////////////////////////////////////////////////
template<typename V, typename GID>
auto referencer(V&& v, GID const& gid) -> decltype(v[gid])
{
  return v[gid];
}

//////////////////////////////////////////////////////////////////////
/// @brief Returns a reference to the view element at given GID.
/// @internal
//////////////////////////////////////////////////////////////////////
template<typename V, typename... Ts>
auto referencer(V&& v, stapl::tuple<Ts...> const& gid,
                typename std::enable_if<
                           is_view<typename std::decay<V>::type>::value
                         >::type* = 0)
  -> decltype(v(Ts()...))
{
  using VV = typename std::decay<V>::type;
  using result_type = decltype(v(Ts()...));
  using mem_fun_type = result_type (VV::*)(Ts...) const;

  const mem_fun_type mem_fun = &VV::operator();

  return tuple_ops::apply(std::forward<V>(v), mem_fun, gid);
}

//////////////////////////////////////////////////////////////////////
/// @brief Returns a reference to an element at given GID within a nested
///        container @c v, represented by a proxy of @c V.
///
/// For proxy of lightweight multiarray, this is an optimization that avoids
/// the indirection associated with unpacking the gid tuple by directly calling
/// the proxy's operator[], where the whole tuple is needed to create the member
/// accessor for the nested container (in which the unpacking finally takes
/// place).
///
/// @internal
//////////////////////////////////////////////////////////////////////
template<typename V, typename... Ts>
auto referencer(V&& v, stapl::tuple<Ts...> const& gid,
                typename std::enable_if<
                           is_proxy<typename std::decay<V>::type>::value
                         >::type* = 0)
  -> decltype(v[gid])
{
  return v[gid];
}

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction used to compute whether or not all of the views
///        in a pack have a defined iterator type.
/// @internal
//////////////////////////////////////////////////////////////////////
template<typename... V>
struct pack_has_iterator;

template<typename V0, typename... V>
struct pack_has_iterator<V0, V...>
 : public std::integral_constant<bool,
            (!stapl::is_view<typename std::decay<V0>::type>::value ||
            stapl::has_iterator<V0>::value) && pack_has_iterator<V...>::value
          >
{ };

template<>
struct pack_has_iterator<>
 : public std::true_type
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Assigns the result of a reduction operation to a variable.
///
/// This is needed to provide a uniform interface for handling values that are
/// provided by a @ref proxy or non-proxy reference.
//////////////////////////////////////////////////////////////////////
template<typename Result>
inline
void reduce_assign(Result& lhs, Result const& rhs)
{
  lhs = rhs;
}


//////////////////////////////////////////////////////////////////////
/// @brief Assigns the result of a reduction operation that is represented by a
///   proxy to a variable represented by a proxy.
///
/// This is needed to provide a uniform interface for handling values that are
/// provided by a @ref proxy or non-proxy reference.
//////////////////////////////////////////////////////////////////////
template<typename T, typename A>
inline
void reduce_assign(proxy<T, A>& lhs, proxy<T, A> const& rhs)
{
  proxy_core_access::reset(lhs, rhs);
}

} // namespace helpers
} // namespace optimizers
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_UTILS_HPP
