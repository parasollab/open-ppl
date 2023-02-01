/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_RANDOM_ACCESS_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_RANDOM_ACCESS_HPP

#include <stapl/containers/type_traits/distribution_traits.hpp>
#include <stapl/containers/type_traits/is_container.hpp>
#include <stapl/containers/iterators/container_accessor.hpp>
#include <stapl/containers/iterators/const_container_accessor.hpp>

#include <stapl/views/array_view.hpp>

namespace stapl {

namespace operations {

//////////////////////////////////////////////////////////////////////
/// @brief Functor that creates a reference either as a proxy or a view, based
///   on @ref use_view_as_reference metafunction.
/// @tparam T The value type of the outer container, possibly another container.
/// @tparam Container The outer container type.
/// @tparam GID Gid type of the outer container.
///
/// Primary template constructs a proxy and returns it as the reference for the
/// iterator dereference. The sole specialization instead heap allocates the
/// proxy and uses it as the container for a view (the type of which is
/// determined by @ref container_traits).
//////////////////////////////////////////////////////////////////////
template<typename T, typename Container, typename GID,
         bool = view_impl::use_view_as_reference<T>::value>
struct referencer
{
private:
  typedef container_accessor<Container>        accessor_t;

public:
  typedef proxy<T, accessor_t>                 result_type;

  result_type operator()(Container* ct_ptr, GID const& gid) const
  {
    return result_type(accessor_t(ct_ptr, gid));
  }

  template <typename... Indices>
  result_type operator()(Container* ct_ptr, Indices const&... i) const
  {
    return result_type(accessor_t(ct_ptr, i...));
  }
};


template<typename T, typename Container, typename GID>
struct referencer<T, Container, GID, true>
{
private:
  typedef container_accessor<Container>        accessor_t;
  typedef proxy<T, accessor_t>                 proxy_t;

public:
  typedef typename container_traits<T>::
    template construct_view<proxy_t>::type     result_type;

  result_type operator()(Container* ct_ptr, GID const& gid) const
  {
    return result_type(new proxy_t(accessor_t(ct_ptr, gid)));
  }

  template <typename... Indices>
  result_type operator()(Container* ct_ptr, Indices const&... i) const
  {
    return result_type(new proxy_t(accessor_t(ct_ptr, i...)));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor that creates a const reference either as a proxy or a view,
///    based on @ref use_view_as_reference metafunction.
/// @tparam T The value type of the outer container, possibly another container.
/// @tparam Container The outer container type.
/// @tparam GID Gid type of the outer container.
///
/// Primary template constructs a proxy with a const_accessor and returns it
/// as the reference for the iterator dereference. The sole specialization
/// instead heap allocates the proxy and uses it as the container for a view
/// (the type of which is determined by @ref container_traits).
//////////////////////////////////////////////////////////////////////
template<typename T, typename Container, typename GID,
         bool = view_impl::use_view_as_reference<T>::value>
struct const_referencer
{
private:
  typedef const_container_accessor<Container>  accessor_t;

public:
  typedef proxy<T, accessor_t>                 result_type;

  result_type operator()(Container const* ct_ptr, GID const& gid) const
  {
    return result_type(accessor_t(ct_ptr, gid));
  }
};


template<typename T, typename Container, typename GID>
struct const_referencer<T, Container, GID, true>
{
private:
  typedef const_container_accessor<Container>  accessor_t;
  typedef proxy<T, accessor_t>                 proxy_t;

public:
  typedef typename container_traits<T>::
    template construct_view<proxy_t>::type     result_type;

  result_type operator()(Container const* ct_ptr, GID const& gid) const
  {
    return result_type(new proxy_t(accessor_t(ct_ptr, gid)));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Operations class for container distributions that provides
/// for random access by allowing the creation of references.
///
/// @tparam Derived The most derived distribution class
////////////////////////////////////////////////////////////////////////
template<typename Derived>
class random_access
{
private:
  typedef typename distribution_traits<Derived>::value_type     value_type;
  typedef typename distribution_traits<Derived>::container_type container_t;
  typedef typename distribution_traits<Derived>::gid_type       gid_type;
  typedef referencer<value_type, container_t, gid_type>         referencer_t;
  typedef const_referencer<value_type, container_t, gid_type>
                                                             const_referencer_t;

public:
  typedef typename referencer_t::result_type                    reference;
  typedef typename const_referencer_t::result_type              const_reference;

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the element at a specific gid.
  /// @param gid The index for which to create the reference
  /// @return A proxy over the element.
  ////////////////////////////////////////////////////////////////////////
  reference operator[](gid_type const& gid)
  {
    return referencer_t()(static_cast<Derived&>(*this).container(), gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the element at a specific gid.
  /// @param gid The index for which to create the reference
  /// @return A proxy over the element.
  ////////////////////////////////////////////////////////////////////////
  const_reference operator[](gid_type const& gid) const
  {
    return referencer_t()(static_cast<Derived&>(*this).container(), gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the element at a specific gid.
  /// @param gid The GID of the element
  /// @return Proxy over the element.
  //////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& gid)
  {
    return operator[](gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the element at a specific gid.
  /// @param gid The GID of the element
  /// @return Proxy over the element.
  //////////////////////////////////////////////////////////////////////
  const_reference make_reference(gid_type const& gid) const
  {
    return operator[](gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the element at a specific gid.
  /// @param i The indices of the gid at which the element resides.
  /// @return Proxy over the element.
  //////////////////////////////////////////////////////////////////////
  template<typename... Indices>
  reference make_reference(Indices const&... i)
  {
    return referencer_t()(static_cast<Derived&>(*this).container(), i...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the element at a specific gid.
  /// @param i The indices of the gid at which the element resides.
  /// @return Proxy over the element.
  //////////////////////////////////////////////////////////////////////
  template<typename... Indices>
  const_reference make_reference(Indices const&... i) const
  {
    return referencer_t()(static_cast<Derived&>(*this).container(), i...);
  }
}; // class random_access

} // namespace operations

} // namespace stapl

#endif
