/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MULTIMAP_HPP
#define STAPL_CONTAINERS_UNORDERED_MULTIMAP_HPP

#include <stapl/containers/base/container.hpp>
#include <stapl/containers/unordered_multimap/unordered_multimap_traits.hpp>
#include <stapl/views/map_view.hpp>

#include <stapl/utility/use_default.hpp>
#include "proxy.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Template specialization of container_traits for
///   the unordered_multimap. It is used to specify types that can customize the
///   behavior of the container, including the hash function used and the
///   partitioning and distribution of elements stored in the container.
/// @ingroup punorderedmultimapTraits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Mapped, typename ...OptionalParams>
struct container_traits<unordered_multimap<Key, Mapped, OptionalParams...> >
  : public unordered_multimap_impl::compute_unordered_multimap_traits<
      Key, Mapped, OptionalParams...>::type
{
  typedef typename
    unordered_multimap_impl::compute_unordered_multimap_traits<
      Key, Mapped, OptionalParams...
    >::type                                             traits_t;

  typedef typename traits_t::base_container_type        base_container_type;
  typedef typename traits_t::directory_type             directory_type;
  typedef typename traits_t::container_manager_type     container_manager_type;
  typedef typename traits_t::manager_type               manager_type;

  typedef Key                                           key_type;
  typedef Mapped                                        mapped_type;
  typedef multi_key<Key>                                gid_type;
  typedef std::pair<const gid_type, mapped_type>        value_type;
  typedef value_type                                    stored_type;
  typedef typename traits_t::hasher                     base_hasher;
  typedef typename traits_t::key_equal                  key_equal;
  typedef typename traits_t::partition_type             partition_type;
  typedef typename traits_t::mapper_type                mapper_type;
  typedef typename traits_t::domain_type                domain_type;
};


template<typename Key, typename Mapped, typename ...OptionalParams>
class unordered_multimap
  : public container<unordered_multimap<Key, Mapped, OptionalParams...> >
{
public:
  STAPL_IMPORT_TYPE(typename container_traits<unordered_multimap>, base_hasher)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_multimap>, key_equal)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_multimap>,
    partition_type)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_multimap>, mapper_type)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_multimap>,
    container_manager_type)
  STAPL_IMPORT_TYPE(typename container_traits<unordered_multimap>, manager_type)

private:
  typedef container<unordered_multimap>                 base_type;

public:
  typedef Key                                           key_type;
  typedef Mapped                                        mapped_type;
  typedef multi_key<key_type>                           gid_type;
  typedef std::pair<const gid_type, mapped_type>        value_type;
  typedef value_type                                    stored_type;
  typedef std::pair<key_type, mapped_type>              element_type;
  typedef pair_hash<Key, base_hasher>                   hasher;
  typedef size_t                                        size_type;
  typedef typename base_type::distribution_type         distribution_type;
  typedef typename distribution_type::reference         reference;
  typedef typename distribution_type::const_reference   const_reference;
  typedef typename distribution_type::second_reference  second_reference;
  typedef typename distribution_type::iterator          iterator;
  typedef typename distribution_type::const_iterator    const_iterator;
  typedef typename distribution_type::directory_type    directory_type;
  typedef typename distribution_type::loc_dist_metadata loc_dist_metadata;
  typedef typename mapper_type::domain_type             map_dom_t;
  typedef iterator_domain<
            distribution_type,
            domain_impl::f_deref_gid<gid_type> >        domain_type;
  typedef map_view<unordered_multimap>                  view_type;

public:
  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an unordered multimap container.
  /// @param hash The hashing function used for distribution.
  /// @param comp The key used for associating elements
  /// @bug get_num_locations should not be called, it needs to be
  /// resolved in order to use the unordered_multimap in nested_containers.
  //////////////////////////////////////////////////////////////////////
  unordered_multimap(hasher const& hash = hasher(),
                       key_equal const& comp = key_equal())
    : base_type(
        directory_type(
          manager_type(
            partition_type(), mapper_type(map_dom_t(get_num_locations()))
          )
        ),
        container_manager_type(
          partition_type(), mapper_type(map_dom_t(get_num_locations())),
          hash, comp
        )
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an unordered multimap container given a
  /// partition.
  /// @param partitioner The partition.
  /// @param hash The hashing function used for distribution.
  /// @param comp The key used for associating elements.
  //////////////////////////////////////////////////////////////////////
  unordered_multimap(partition_type const& part, hasher const& hash = hasher(),
                       key_equal const& comp = key_equal())
    : base_type(
        directory_type(
          part, mapper_type(part.domain()),
          manager_type(part, mapper_type(part.domain()))
        ),
        container_manager_type(part, mapper_type(part.domain()),hash,comp)
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an unordered multimap container given a partition
  /// strategy and a mapper for the distribution.
  /// @param partitioner The partition.
  /// @param mapper The mapper used for the distribution of elements.
  /// @param hash The hashing function used for distribution.
  /// @param comp The key used for associating elements
  //////////////////////////////////////////////////////////////////////
  unordered_multimap(partition_type const& partitioner,
                     mapper_type const& mapper, hasher const& hash = hasher(),
                     key_equal const& comp = key_equal())
    : base_type(
        directory_type(
          partitioner, mapper, manager_type(partitioner, mapper)
        ),
        container_manager_type(partitioner, mapper,hash, comp)
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief The copy constructor
  /// @param other The container to copy
  //////////////////////////////////////////////////////////////////////
  unordered_multimap(unordered_multimap const& other)
    : base_type(other)
  { }

  /// @}

  /// @name Element Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to insert an element into the container.
  /// @param val The pair of <key_type,mapped_type> to be inserted.
  /// @todo: Verify if this function should return a value with
  ///   future_exist<bool> or similar.
  //////////////////////////////////////////////////////////////////////
  void insert(element_type const& val)
  {
    this->incr_version();
    this->distribution().insert(
      value_type(gid_type(val.first), val.second), true);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to insert an element into the container.
  /// @param key The key of type key_type to be inserted.
  /// @param val The mapped element of type mapped_type to be inserted.
  /// @todo: Verify if this function should return a value with
  ///   future_exist<bool> or similar.
  //////////////////////////////////////////////////////////////////////
  void insert(key_type const& key, mapped_type const& val)
  {
    this->incr_version();
    this->distribution().insert(value_type(gid_type(key), val), true);
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
  /// @brief Removes any elements from the container with the key @p k.
  /// @param k The key of the elements to be removed.
  /// @return The number of elements erased.
  //////////////////////////////////////////////////////////////////////
  size_t erase_sync(key_type const& k)
  {
    this->incr_version();
    return this->distribution().erase_sync(gid_type(k));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear the container by removing all the elements in it.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->incr_version();
    this->distribution().clear();
  }

  /// @}

  /// @name Element Access
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to access an element with key @P k.
  /// @param k The key of the element to access.
  /// @return A proxy to the element
  //////////////////////////////////////////////////////////////////////
  reference make_reference(key_type const& k) const
  {
    return this->distribution().make_reference(gid_type(k));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to access an element with the key @p k.
  /// @param k The key of the element to access.
  /// @return A proxy to the element
  //////////////////////////////////////////////////////////////////////
  const_reference make_const_reference(key_type const& k) const
  {
    return this->distribution().make_const_reference(gid_type(k));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator over an element.
  /// @param key The key of type key_type to which the iterator should point
  /// @return the Iterator over the element
  /// @todo  specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(key_type const& key)
  {
    return this->distribution().make_iterator(gid_type(key,0));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator over an element.
  /// @param gid The GID of the element to which the iterator should point at
  /// @return the Iterator over the element
  /// @todo  specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return this->distribution().make_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator over an element.
  /// @param key The Key of type key_type on which the iterator should point at
  /// @return the Iterator over the element
  /// @todo  specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  const_iterator make_const_iterator(key_type const& key) const
  {
    return this->distribution().make_const_iterator(gid_type(key,0));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator over an element.
  /// @param gid The GID of type gid_type on which the iterator should point at
  /// @return the Iterator over the element
  /// @todo  specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  const_iterator make_const_iterator(gid_type const& gid) const
  {
    return this->distribution().make_const_iterator(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to the first element in the domain.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return this->distribution().begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator to one past the last element in the domain.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
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
  /// @brief Return an iterator over an element.
  /// @param key The key of type key_type to which the iterator should point
  /// @return An iterator over the element
  /// @todo  specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  iterator find(key_type const& k)
  {
    return this->distribution().find(gid_type(k, 0));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator over an element.
  /// @param key The key of type key_type to which the iterator should point
  /// @return the Iterator over the element
  /// @todo  specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  const_iterator find(key_type const& k) const
  {
    return this->distribution().find(gid_type(k, 0));
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
  bool empty(void) const
  {
    return (this->size()==0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return the domain of the container
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
    return this->distribution().hash_function();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an instance of the container's comparator function
  //////////////////////////////////////////////////////////////////////
  key_equal key_eq(void) const
  {
    return this->distribution().key_eq();
  }

  /// @}
}; // unordered_multimap

} // namespace stapl

#endif
