/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_ITERABLE_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_ITERABLE_HPP

#include <stapl/containers/type_traits/distribution_traits.hpp>
#include <stapl/containers/iterators/container_accessor.hpp>
#include <stapl/containers/iterators/const_container_accessor.hpp>
#include <stapl/containers/iterators/container_iterator.hpp>
#include <stapl/containers/iterators/const_container_iterator.hpp>

namespace stapl {

namespace operations {

//////////////////////////////////////////////////////////////////////
/// @brief Operations class for container distributions that provides
/// global iteration.
///
/// @tparam Derived The most derived distribution class
/// @todo This operation for now assumes the iterator type of the container
/// is always @ref container_iterator, as it is currently not possible to grab
/// the true iterator type from the container because it is not fully
/// defined at this point.
////////////////////////////////////////////////////////////////////////
template<typename Derived>
class iterable
{
private:
  typedef Derived                                           derived_type;
  typedef distribution_traits<derived_type>                 traits_t;
  typedef typename traits_t::container_type                 container_t;

public:

  typedef typename std::random_access_iterator_tag          iterator_category;

  typedef typename traits_t::gid_type                       gid_type;
  typedef container_accessor<container_t>                   accessor_type;
  typedef const_container_accessor<container_t>             const_accessor_type;
  typedef container_iterator<container_t, accessor_type,
                             iterator_category
                            >                               iterator;
  typedef const_container_iterator<
    container_t, const_accessor_type, iterator_category
  >                                                         const_iterator;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Cast this object to its most derived class. Used for CRTP.
  //////////////////////////////////////////////////////////////////////
  derived_type& derived(void)
  {
    return static_cast<derived_type&>(*this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Cast this object to a const of its most derived class.
  /// Used for CRTP.
  //////////////////////////////////////////////////////////////////////
  derived_type const& derived(void) const
  {
    return static_cast<derived_type const&>(*this);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to a specific GID of the container.
  /// @param gid The GID for which to create the iterator
  /// @return An iterator to the value at gid
  ////////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return iterator(derived().container(), gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to a specific GID of the container.
  /// @param gid The GID for which to create the const_iterator
  /// @return A const_iterator to the value at gid
  ////////////////////////////////////////////////////////////////////////
  const_iterator make_const_iterator(gid_type const& gid) const
  {
    return const_iterator(derived().container(), gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to the beginning of the container.
  /// @return A global iterator to the first GID in the domain
  ////////////////////////////////////////////////////////////////////////
  iterator begin()
  {
    return make_iterator(derived().domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to the beginning of the container.
  /// @return A global const_iterator to the first GID in the domain
  ////////////////////////////////////////////////////////////////////////
  const_iterator begin() const
  {
    return make_const_iterator(derived().domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to the beginning of the container.
  /// @return A global const_iterator to the first GID in the domain
  ////////////////////////////////////////////////////////////////////////
  const_iterator cbegin() const
  {
    return make_const_iterator(derived().domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to the end of the container.
  /// @return A global iterator to the one past the last GID in the domain
  ////////////////////////////////////////////////////////////////////////
  iterator end()
  {
    return make_iterator(derived().domain().open_last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to the end of the container.
  /// @return A global const_iterator to the one past the last GID in the domain
  ////////////////////////////////////////////////////////////////////////
  const_iterator end() const
  {
    return make_const_iterator(derived().domain().open_last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_iterator to the end of the container.
  /// @return A global const_iterator to the one past the last GID in the domain
  ////////////////////////////////////////////////////////////////////////
  const_iterator cend() const
  {
    return make_const_iterator(derived().domain().open_last());
  }
}; // class iterable

} // namespace operations

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_ITERABLE_HPP
