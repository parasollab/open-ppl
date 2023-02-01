
/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_PROXY_AS_REFERENCE_HPP
#define STAPL_VIEWS_PROXY_AS_REFERENCE_HPP

namespace stapl {

template<typename C>
class local_accessor_graph;

template<typename C>
class local_accessor;

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper for extracting a reference from a local accssor
//////////////////////////////////////////////////////////////////////
template<typename Accessor>
struct as_reference_impl
{
  // This will always fail, because sizeof(T)==1 for an empty class T
  static_assert(sizeof(Accessor) == 0,
                "Cannot get proxy as raw reference for this accessor");
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for local accessor
//////////////////////////////////////////////////////////////////////
template<typename C>
struct as_reference_impl<local_accessor<C>>
{
  using type = typename C::value_type&;

  template<typename Accessor>
  static type get(Accessor&& a)
  {
    return *std::forward<Accessor>(a).m_itr;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for the graph's local accessor
//////////////////////////////////////////////////////////////////////
template<typename C>
struct as_reference_impl<local_accessor_graph<C>>
{
  using type = typename C::value_type&;

  template<typename Accessor>
  static type get(Accessor&& a)
  {
    return std::forward<Accessor>(a).ref();
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Extract the raw reference that a proxy is encapsulating.
///        This is only defined for local accessors.
///
/// @todo This should be removed when we change the semantics of the
///        apply_set and apply_get container methods.
///        See GForge task #1531.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Accessor>
T& as_reference(proxy<T, Accessor> p)
{
  return detail::as_reference_impl<Accessor>::get(
    proxy_core_access::accessor(p));
}

} // namespace stapl

#endif