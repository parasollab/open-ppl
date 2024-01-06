/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_STATIC_ARRAY_HPP
#define STAPL_CONTAINERS_STATIC_ARRAY_HPP

#include <stapl/containers/base/container.hpp>
#include <stapl/containers/array/static_array_traits.hpp>
#include <stapl/containers/distribution/static_array_metadata.hpp>

#include <stapl/views/array_view.hpp>

#include "static_array_fwd.hpp"
#include "static_proxy.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for static_array.
/// @ingroup parrayTraits
/// @see static_array container_traits
/// @todo enable_view_reference typedef is temporary until all containers
///   are updated to reflect views for nested container references.
//////////////////////////////////////////////////////////////////////
template<typename T,  typename... OptionalNoInitParam>
struct container_traits<static_array<T, OptionalNoInitParam...>>
  : public static_array_traits<T, OptionalNoInitParam...>
{
  using enable_view_reference = void;

  template<typename C>
  struct construct_view
  { using type = array_view<C>; };
};


//////////////////////////////////////////////////////////////////////
/// @brief Parallel non-migratable array container.
/// @ingroup parray
///
/// This container provides fast access, but is limited in numerous ways:
/// this container cannot be migrated; the distribution is fixed to a
/// balanced partition with one base container per location.
///
/// @tparam T Type of the stored elements in the container. T must be
/// default assignable, copyable and assignable.
///
/// @tparam OptionalNoInitParam Optionally, the type tag @ref no_initialization
/// can be passed so that the base will attempt to elide default initialization
/// of the container's elements.
////////////////////////////////////////////////////////////////////////
template<typename T, typename... OptionalNoInitParam>
class static_array
  : public container<static_array<T, OptionalNoInitParam...>>
{
public:
  typedef static_array_traits<T, OptionalNoInitParam...>    traits_type;

private:
  typedef container<static_array>                           base_type;
  typedef typename traits_type::partition_type              partition_type;
  typedef typename traits_type::mapper_type                 mapper_type;
  typedef typename mapper_type::domain_type                 mapper_domain_type;

public:
  typedef typename base_type::distribution_type             distribution_type;

  typedef T                                                 value_type;
  typedef typename traits_type::gid_type                    gid_type;
  typedef gid_type                                          index_type;
  typedef typename traits_type::domain_type                 domain_type;
  typedef typename domain_type::size_type                   size_type;

  typedef typename distribution_type::reference             reference;
  typedef typename distribution_type::const_reference       const_reference;
  typedef typename distribution_type::iterator              iterator;
  typedef typename distribution_type::const_iterator        const_iterator;

  typedef static_array_metadata<distribution_type>          loc_dist_metadata;

  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with a given size and
  /// default constructs all elements.
  /// @param n The size of the array
  ////////////////////////////////////////////////////////////////////////
  static_array(size_type n)
    : base_type(partition_type(domain_type(n), get_num_locations()),
                mapper_type(mapper_domain_type(get_num_locations())))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with a given size and
  /// constructs all elements with a default value.
  /// @param n The size of the array
  /// @param default_value The initial value of the elements in the container
  ////////////////////////////////////////////////////////////////////////
  static_array(size_type n, value_type const& default_value)
    : base_type(partition_type(domain_type(n), get_num_locations()),
                mapper_type(mapper_domain_type(get_num_locations())),
                default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array with a given size and default
  /// value where the value type of the container is itself a parallel
  /// container.
  /// @param n The size of the array
  /// @param default_value The initial value of the elements in the container
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers, in the context of containers of containers.
  ////////////////////////////////////////////////////////////////////////
  template <typename DP>
  static_array(size_type n,
               value_type const& default_value,
               DP const& dis_policy)
    : base_type(partition_type(domain_type(n),
                               get_num_locations()),
                mapper_type(mapper_domain_type(get_num_locations())),
                default_value, dis_policy)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array of arrays with given
  /// n-dimensional size.
  /// @param dims A cons list specifying the dimensions of the containers.
  ////////////////////////////////////////////////////////////////////////
  template <typename X, typename Y>
  static_array(boost::tuples::cons<X,Y> dims)
    : base_type(partition_type(domain_type(dims.get_head()),
                               get_num_locations()),
                mapper_type(mapper_domain_type(get_num_locations())),
                dims)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an array of arrays with given
  /// n-dimensional size.
  /// @param dims A cons list specifying the dimensions of the containers.
  /// @param dis_policy A distribution policy that specifies how to distribute
  /// the nested containers.
  ////////////////////////////////////////////////////////////////////////
  template <typename X, typename Y, typename DP>
  static_array(boost::tuples::cons<X,Y> dims, DP const& dis_policy)
    : base_type(partition_type(domain_type(dims.get_head()),
                               get_num_locations()),
                mapper_type(mapper_domain_type(get_num_locations())),
                dims, dis_policy)
  { }

  /// @}

  /// @name Element Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to a specific index of the array.
  /// @param gid The index for which to create the reference
  /// @return A proxy of the value at gid
  ////////////////////////////////////////////////////////////////////////
  reference operator[](gid_type const& gid)
  {
    return this->distribution().operator[](gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_reference to a specific index of the array.
  /// @param gid The index for which to create the reference
  /// @return A proxy of the value at gid
  ////////////////////////////////////////////////////////////////////////
  const_reference operator[](gid_type const& gid) const
  {
    return this->distribution().operator[](gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the first element of the array.
  /// @return A proxy of the value of the element at the first index.
  ////////////////////////////////////////////////////////////////////////
  reference front()
  {
    return this->distribution().operator[](this->domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_reference to the first element of the array.
  /// @return A proxy of the value of the element at the first index.
  ////////////////////////////////////////////////////////////////////////
  const_reference front() const
  {
    return this->distribution().operator[](this->domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the last element of the array.
  /// @return A proxy of the value of the element at the last index.
  ////////////////////////////////////////////////////////////////////////
  reference back()
  {
    return this->distribution().operator[](this->domain().last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_reference to the last element of the array.
  /// @return A proxy of the value of the element at the last index.
  ////////////////////////////////////////////////////////////////////////
  const_reference back() const
  {
    return this->distribution().operator[](this->domain().last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to a specific index of the array.
  /// @param gid The index for which to create the reference
  /// @return A proxy of the value at gid
  ////////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& gid)
  {
    return this->distribution().make_reference(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_reference to a specific index of the array.
  /// @param gid The index for which to create the reference
  /// @return A proxy of the value at gid
  ////////////////////////////////////////////////////////////////////////
  const_reference make_reference(gid_type const& gid) const
  {
    return this->distribution().make_reference(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to the beginning of the array.
  /// @return A global iterator to GID 0
  ////////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return this->distribution().make_iterator(
      this->distribution().domain().first()
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an const_iterator to the beginning of the array.
  /// @return A global const_iterator to GID 0
  ////////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return this->distribution().make_const_iterator(
      this->distribution().domain().first()
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an const_iterator to the beginning of the array.
  /// @return A global const_iterator to GID 0
  ////////////////////////////////////////////////////////////////////////
  const_iterator cbegin(void) const
  {
    return this->distribution().make_const_iterator(
      this->distribution().domain().first()
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to one past the end of the array.
  /// @return A global iterator of the end
  ////////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return this->distribution().make_iterator(
      this->distribution().domain().last() + 1
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an const_iterator to one past the end of the array.
  /// @return A global const_iterator of the end
  ////////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return this->distribution().make_const_iterator(
      this->distribution().domain().last() + 1
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an const_iterator to one past the end of the array.
  /// @return A global const_iterator of the end
  ////////////////////////////////////////////////////////////////////////
  const_iterator cend(void) const
  {
    return this->distribution().make_const_iterator(
      this->distribution().domain().last() + 1
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to a specific index of the array.
  /// @param index The index for which to create the iterator
  /// @return An iterator to the value at index
  ////////////////////////////////////////////////////////////////////////
  iterator make_iterator(index_type const& index)
  {
    return this->distribution().make_iterator(index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an iterator to a specific index of the array.
  /// @param index The index for which to create the iterator
  /// @return An iterator to the value at index
  ////////////////////////////////////////////////////////////////////////
  const_iterator make_iterator(index_type const& index) const
  {
    return this->distribution().make_iterator(index);
  }

  /// @}
}; // class static_array

} // namespace stapl

#endif // STAPL_CONTAINERS_STATIC_ARRAY_HPP
