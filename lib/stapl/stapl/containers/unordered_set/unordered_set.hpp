/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_SET_HPP
#define STAPL_CONTAINERS_UNORDERED_SET_HPP

#include <stapl/containers/base/container.hpp>
#include <stapl/containers/unordered_set/unordered_set_traits.hpp>
#include <stapl/views/set_view.hpp>
#include "proxy.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Template specialization of container_traits for
///   the unordered_set. It is used to specify types that can customize the
///   behavior of the container, including the hash function used and the
///   partitioning and distribution of elements stored in the container.
/// @ingroup punorderedsetTraits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename ...OptionalParams>
struct container_traits<unordered_set<Key, OptionalParams...> >
  : public unordered_set_impl::compute_unordered_set_traits<
      Key, OptionalParams...>::type
{
  typedef typename
    unordered_set_impl::compute_unordered_set_traits<
      Key, OptionalParams...>::type                      traits_t;

  typedef typename traits_t::base_container_type         base_container_type;
  typedef typename traits_t::directory_type              directory_type;
  typedef typename traits_t::container_manager_type      container_manager_type;
  typedef typename traits_t::manager_type                manager_type;

  typedef Key                                            value_type;
  typedef Key                                            mapped_type;
  typedef typename traits_t::gid_type                    gid_type;
  typedef typename traits_t::hasher                      hasher;
  typedef typename traits_t::key_equal                   key_equal;
  typedef typename traits_t::partition_type              partition_type;
  typedef typename traits_t::mapper_type                 mapper_type;
  typedef typename traits_t::domain_type                 domain_type;

  typedef void enable_view_reference;

  template <typename C>
  struct construct_view
  {
    typedef set_view<C> type;
  };
};


template<typename Key, typename ...OptionalParams>
class unordered_set
  : public container<unordered_set<Key, OptionalParams...> >
{
public:
  STAPL_IMPORT_TYPE(typename container_traits<unordered_set>, hasher)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_set>, key_equal)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_set>, partition_type)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_set>, mapper_type)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_set>,
    container_manager_type)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_set>, manager_type)

private:
  typedef container<unordered_set>                       base_type;

public:
  typedef Key                                            key_type;
  typedef Key                                            mapped_type;
  typedef Key                                            value_type;
  typedef Key                                            stored_type;
  typedef Key                                            gid_type;
  typedef size_t                                         size_type;
  typedef typename base_type::distribution_type          distribution_type;
  typedef typename distribution_type::reference          reference;
  typedef typename distribution_type::const_reference    const_reference;
  typedef typename distribution_type::iterator           iterator;
  typedef typename distribution_type::const_iterator     const_iterator;
  typedef typename distribution_type::directory_type     directory_type;
  typedef typename distribution_type::loc_dist_metadata  loc_dist_metadata;
  typedef typename mapper_type::domain_type              map_dom_t;
  typedef iterator_domain<distribution_type>             domain_type;
  typedef set_view<unordered_set>                        view_type;

  /// @name Constructors
  /// @{

  unordered_set(hasher const& hash = hasher(),
                key_equal const& comp = key_equal())
    : base_type(
        directory_type(
          manager_type(
            partition_type(hash), mapper_type(map_dom_t(get_num_locations()))
          )
        ),
        container_manager_type(partition_type(hash),
          mapper_type(map_dom_t(get_num_locations())), hash, comp
        )
      )
  { }

  unordered_set(partition_type const& part, hasher const& hash = hasher(),
                     key_equal const& comp = key_equal())
    : base_type(
        directory_type(
          part, mapper_type(part.domain()),
          manager_type(part, mapper_type(part.domain()))
        ),
        container_manager_type(part,
          mapper_type(map_dom_t(get_num_locations())), hash, comp
        )
      )
  { }

  unordered_set(partition_type const& partitioner, mapper_type const& mapper)
    : base_type(partitioner, mapper)
  { }

  unordered_set(unordered_set const& other)
    : base_type(other)
  { }

  /// @}

  /// @name Element Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to insert a key in the container.
  /// @param k The key to be inserted.
  /// @todo: Verify if this function should return a value with
  ///   future_exist<bool> or similar.
  //////////////////////////////////////////////////////////////////////
  void insert(key_type const& k)
  {
    this->incr_version();
    this->distribution().insert(k);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove an element from the container asynchronously.
  /// @param k The Key of the element to be removed.
  //////////////////////////////////////////////////////////////////////
  void erase(key_type const& k)
  {
    this->incr_version();
    this->distribution().erase(k);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove an element from the container.
  /// @param k The key to be removed.
  /// @return The number of elements removed.
  //////////////////////////////////////////////////////////////////////
  size_t erase_sync(key_type const& k)
  {
    this->incr_version();
    return this->distribution().erase_sync(k);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear the container by removing all the elements in it.
  //////////////////////////////////////////////////////////////////////
  void clear()
  {
    this->incr_version();
    this->distribution().clear();
  }

  /// @}

  /// @name Element Access
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator to the first element in the domain.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return this->distribution().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator to one past the last element in the domain.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return this->distribution().end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator to the first element in the domain.
  //////////////////////////////////////////////////////////////////////
  const_iterator cbegin(void) const
  {
    return this->distribution().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator to one past the last element in the domain.
  //////////////////////////////////////////////////////////////////////
  const_iterator cend(void) const
  {
    return this->distribution().end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator over a key.
  /// @param key The Key of type key_type on which the iterator should point at
  /// @return the Iterator over the key
  /// @todo  specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  const_iterator find(key_type const& key) const
  {
    return this->distribution().find(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to access a specific key.
  /// @param k The key of the element to access.
  /// @return A proxy to the key
  //////////////////////////////////////////////////////////////////////
  const_reference make_const_reference(key_type const& k) const
  {
    return this->distribution().make_const_reference(k);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to access a specific key.
  /// @param k The key of the element to access.
  /// @return A proxy to the key
  //////////////////////////////////////////////////////////////////////
  reference make_reference(key_type const& k) const
  {
    return this->distribution().make_reference(k);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::unordered_map::make_const_iterator(key_type const& key)
  //////////////////////////////////////////////////////////////////////
  const_iterator make_const_iterator(key_type const& key) const
  {
    return this->distribution().make_const_iterator(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::map::make_iterator(key_type const& key)
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(key_type const& key) const
  {
    return this->distribution().make_const_iterator(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in the container with the key
  ///   @p key.
  /// @param key The key to count
  /// @return The number of elements with associated with @key
  //////////////////////////////////////////////////////////////////////
  size_t count(key_type const& key) const
  {
    return this->distribution().count(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the bounds of a range of elements with keys equal
  ///   to @p k.
  /// @param k The target key
  /// @return A view with a domain restricted to the range of keys
  ///   equivalent to @p k.
  //////////////////////////////////////////////////////////////////////
  view_type equal_range(key_type const& k)
  {
    if (this->distribution().equal_range(k)!=index_bounds<key_type>::invalid())
    {
      domain_type dom(k, k, this->distribution(), 1);
      return view_type(*this, dom);
    }

    domain_type empty_dom(gid_type(), gid_type(), this->distribution(), 0);
    return view_type(*this, empty_dom);
  }

  /// @}

  /// @name Memory and Domain Management
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of elements in the container.
  ///
  /// This method is one-sided, If other locations may be concurrently
  /// performing operations that change their local size and the effects
  /// are desired to be observed in a deterministic way, then appropriate
  /// synchronization, e.g. a fence, may be required before or after the
  /// call to size, to enforce appropriate ordering.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->distribution().size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return if the container is empty or not.
  /// @return a bool set to true if it is empty, false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return (this->distribution().size()==0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return the domain of the distribution
  //////////////////////////////////////////////////////////////////////
  domain_type domain(void) const
  {
    return domain_type(this->distribution());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an instance of the container's hash function
  //////////////////////////////////////////////////////////////////////
  hasher hash_function(void) const
  {
    return hasher();//this->distribution().hash_function();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an instance of the container's comparator function
  //////////////////////////////////////////////////////////////////////
  key_equal key_eq(void) const
  {
    return key_equal();//this->distribution().key_eq();
  }

  /// @}
}; // unordered_set

} // namespace stapl

#endif
