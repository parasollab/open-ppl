/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MULTISET_HPP
#define STAPL_CONTAINERS_UNORDERED_MULTISET_HPP

#include <stapl/containers/base/container.hpp>
#include <stapl/containers/unordered_multiset/unordered_multiset_traits.hpp>
#include <stapl/views/set_view.hpp>

#include <stapl/utility/use_default.hpp>
#include "proxy.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Template specialization of container_traits for
///   the unordered_multiset. It is used to specify types that can customize the
///   behavior of the container, including the hash function used and the
///   partitioning and distribution of elements stored in the container.
/// @ingroup punorderedmultisetTraits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename ...OptionalParams>
struct container_traits<unordered_multiset<Key, OptionalParams...> >
  : public unordered_multiset_impl::compute_unordered_multiset_traits<
      Key, OptionalParams...>::type
{
  typedef typename
    unordered_multiset_impl::compute_unordered_multiset_traits<
      Key, OptionalParams...
    >::type                                             traits_t;

  typedef typename traits_t::base_container_type        base_container_type;
  typedef typename traits_t::directory_type             directory_type;
  typedef typename traits_t::container_manager_type     container_manager_type;
  typedef typename traits_t::domain_type                domain_type;

  typedef typename traits_t::key_type                   key_type;
  typedef key_type                                      mapped_type;
  typedef typename traits_t::value_type                 value_type;
  typedef typename traits_t::gid_type                   gid_type;
  typedef typename traits_t::hasher                     hasher;
  typedef typename traits_t::key_equal                  key_equal;
  typedef typename traits_t::partition_type             partition_type;
  typedef typename traits_t::mapper_type                mapper_type;

  typedef void enable_view_reference;

  template <typename C>
  struct construct_view
  {
    typedef set_view<C> type;
  };
};


template<typename Key, typename ...OptionalParams>
class unordered_multiset
  : public container<unordered_multiset<Key, OptionalParams...> >
{
public:
  STAPL_IMPORT_TYPE(typename container_traits<unordered_multiset>, hasher)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_multiset>, key_equal)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_multiset>,
    partition_type)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_multiset>, mapper_type)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_multiset>,
    container_manager_type)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_multiset>, manager_type)

private:
  typedef container<unordered_multiset>                 base_type;

public:
  typedef Key                                           key_type;
  typedef Key                                           mapped_type;
  typedef Key                                           value_type;
  typedef multi_key<Key>                                stored_type;
  typedef stored_type                                   gid_type;
  typedef size_t                                        size_type;
  typedef typename base_type::distribution_type         distribution_type;
  typedef typename distribution_type::reference         reference;
  typedef typename distribution_type::iterator          iterator;
  typedef typename distribution_type::const_reference   const_reference;
  typedef typename distribution_type::const_iterator    const_iterator;
  typedef typename distribution_type::directory_type    directory_type;
  typedef typename distribution_type::loc_dist_metadata loc_dist_metadata;
  typedef typename mapper_type::domain_type             map_dom_t;
  typedef iterator_domain<
            distribution_type,
            domain_impl::f_deref_gid<gid_type>
          >                                             domain_type;
  typedef set_view<unordered_multiset>                  view_type;

  /// @name Constructors
  /// @{

  /////////////////////////////////////////////////////////////////////////////
  /// @bug get_num_locations should not be called, it needs to be resolved in
  ///   order to use the unordered_set in nested_containers.
  /////////////////////////////////////////////////////////////////////////////
  unordered_multiset(hasher const& hash = hasher(),
                     key_equal const& comp = key_equal())
    : base_type(
        directory_type(
          manager_type(
            partition_type(), mapper_type(map_dom_t(get_num_locations()))
          )
        ),
        container_manager_type(
          partition_type(),
          mapper_type(map_dom_t(get_num_locations())), hash, comp
        )
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an unordered multiset container given a
  /// partition.
  /// @param partitioner The partition.
  /// @param hash The hashing function used for distribution.
  /// @param comp The key used for associating elements.
  //////////////////////////////////////////////////////////////////////
  unordered_multiset(partition_type const& part, hasher const& hash = hasher(),
                     key_equal const& comp = key_equal())
    : base_type(
        directory_type(
          part, mapper_type(part.domain()),
          manager_type(part, mapper_type(part.domain()))
        ),
        container_manager_type(part, mapper_type(part.domain()), hash, comp)
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an unordered multiset container given a partition
  /// strategy and a mapper for the distribution.
  /// @param partitioner The partition.
  /// @param mapper The mapper used for the distribution of elements.
  /// @param hash The hashing function used for distribution.
  /// @param comp The key used for associating elements
  //////////////////////////////////////////////////////////////////////
  unordered_multiset(partition_type const& part, mapper_type const& mapper,
    hasher const& hash = hasher(),key_equal const& comp = key_equal())
    : base_type(
        directory_type(part, mapper, manager_type(part, mapper)),
        container_manager_type(part,mapper,hash,comp)
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief The copy constructor
  /// @param other The container to copy
  //////////////////////////////////////////////////////////////////////
  unordered_multiset(unordered_multiset const& other)
    : base_type(other)
  { }

  /// @}

  /// @name Element Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to insert an element into the container.
  /// @param k The key to be inserted.
  /// @todo: Verify if this function should return a value with
  ///   future_exist<bool> or similar.
  //////////////////////////////////////////////////////////////////////
  void insert(key_type const& k)
  {
    this->incr_version();
    this->distribution().insert(gid_type(k));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove an element from the container asynchronously.
  /// @param k The Key of the element to be removed.
  //////////////////////////////////////////////////////////////////////
  void erase(key_type const& k)
  {
    this->incr_version();
    this->distribution().erase(gid_type(k));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove any element from the container with the key @p k.
  /// @param k The key to be removed.
  /// @return The number of elements removed.
  //////////////////////////////////////////////////////////////////////
  size_t erase_sync(key_type const& k)
  {
    this->incr_version();
    return this->distribution().erase_sync(gid_type(k));
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
  /// @brief Return a const_iterator over an element.
  /// @param key The Key of type key_type on which the iterator should point at
  /// @return An iterator over the element
  /// @todo  specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  const_iterator find(key_type const& k) const
  {
    return this->distribution().find(k);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator over an element.
  /// @param gid The GID of type gid_type on which the iterator should point at
  /// @return An iterator over the element
  /// @todo  specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  const_iterator find(gid_type const& gid) const
  {
    return this->distribution().find(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to the first element in the domain.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void) const
  {
    return this->distribution().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to one past the last element in the domain.
  //////////////////////////////////////////////////////////////////////
  iterator end(void) const
  {
    return this->distribution().end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator to the first element in the domain.
  //////////////////////////////////////////////////////////////////////
  const_iterator cbegin(void) const
  {
    return this->distribution().cbegin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator to one past the last element in the domain.
  //////////////////////////////////////////////////////////////////////
  const_iterator cend(void) const
  {
    return this->distribution().cend();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to access an element.
  /// @param k The key of the element to access.
  /// @return A proxy to the key
  //////////////////////////////////////////////////////////////////////
  reference make_reference(key_type const& k) const
  {
    return this->distribution().make_reference(gid_type(k));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to access an element.
  /// @param k The key of the element to access.
  /// @return A proxy to the key
  //////////////////////////////////////////////////////////////////////
  const_reference make_const_reference(key_type const& k) const
  {
    return this->distribution().make_const_reference(gid_type(k));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::unordered_set::make_iterator(gid_type const& gid)
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid) const
  {
    return this->distribution().make_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::unordered_set::make_const_iterator(gid_type const& gid)
  //////////////////////////////////////////////////////////////////////
  const_iterator make_const_iterator(gid_type const& gid) const
  {
    return this->distribution().make_const_iterator(gid);
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
  /// @brief Returns a view with a restricted domain over the keys
  ///   equivalent to @p key.
  /// @param key The target key
  /// @return A view over the target range.
  //////////////////////////////////////////////////////////////////////
  view_type equal_range(key_type const& key)
  {
    domain_type dom;

    gid_type gid = this->distribution().equal_range(key);

    if (gid != gid_type()) {
      dom = domain_type(gid_type(key),gid,this->distribution(),gid.second+1);
    }

    return {*this, std::move(dom)};
  }

  /// @}

  /// @name Memory and Domain Management
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of elements in the container.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->distribution().size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return if the container is empty or not.
  /// @return a bool set to true if it is empty, false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool empty() const
  {
    return (this->size()==0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return the domain of the container
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    return domain_type(this->distribution());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an instance of the container's hash function
  //////////////////////////////////////////////////////////////////////
  hasher hash_function() const
  {
    return this->distribution().hash_function();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an instance of the container's comparator function
  //////////////////////////////////////////////////////////////////////
  key_equal key_eq() const
  {
    return this->distribution().key_eq();
  }

  /// @}
}; // unordered_multiset

} // namespace stapl

#endif
