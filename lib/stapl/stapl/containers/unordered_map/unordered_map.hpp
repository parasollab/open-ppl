/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MAP_HPP
#define STAPL_CONTAINERS_UNORDERED_MAP_HPP

#include <stapl/containers/base/container.hpp>
#include <stapl/containers/unordered_map/unordered_map_traits.hpp>
#include <stapl/views/map_view.hpp>
#include "proxy.hpp"

#include <boost/bind/protect.hpp>

namespace stapl {

/////////////////////////////////////////////////////////////////////
/// @brief Template specialization of container_traits for
///   the unordered_map. It is used to specify types that can customize the
///   behavior of the container, including the hash function used and the
///   partitioning and distribution of elements stored in the container.
/// @ingroup punorderedmapTraits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Mapped, typename ...OptionalParams>
struct container_traits<unordered_map<Key, Mapped, OptionalParams...> >
  : public unordered_map_impl::compute_unordered_map_traits<
      Key, Mapped, OptionalParams...>::type
{
  typedef typename
    unordered_map_impl::compute_unordered_map_traits<
      Key, Mapped, OptionalParams...>::type              traits_t;

  typedef typename traits_t::base_container_type         base_container_type;
  typedef typename traits_t::directory_type              directory_type;
  typedef typename traits_t::container_manager_type      container_manager_type;
  typedef typename traits_t::manager_type                manager_type;

  typedef Key                                            key_type;
  typedef Mapped                                         mapped_type;
  typedef std::pair<const key_type, mapped_type>         value_type;
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
    typedef map_view<C> type;
  };
};


template<typename Key, typename Mapped, typename ...OptionalParams>
class unordered_map
  : public container<unordered_map<Key, Mapped, OptionalParams...> >
{
  typedef container_traits<unordered_map> traits_type;
  typedef container<unordered_map>        base_type;

public:
  STAPL_IMPORT_TYPE(typename traits_type, hasher)
  STAPL_IMPORT_TYPE(typename traits_type, key_equal)
  STAPL_IMPORT_TYPE(typename traits_type, partition_type)
  STAPL_IMPORT_TYPE(typename traits_type, mapper_type)
  STAPL_IMPORT_TYPE(typename traits_type, manager_type)

  STAPL_IMPORT_TYPE(typename base_type, directory_type)
  STAPL_IMPORT_TYPE(typename base_type, container_manager_type)

  typedef Key                                           key_type;
  typedef Mapped                                        mapped_type;
  typedef std::pair<const Key, Mapped>                  value_type;
  typedef key_type                                      gid_type;
  typedef value_type                                    stored_type;
  typedef size_t                                        size_type;

  STAPL_IMPORT_TYPE(typename base_type, distribution_type)
  STAPL_IMPORT_TYPE(typename distribution_type, reference)
  STAPL_IMPORT_TYPE(typename distribution_type, second_reference)
  STAPL_IMPORT_TYPE(typename distribution_type, iterator)
  STAPL_IMPORT_TYPE(typename distribution_type, const_iterator)
  STAPL_IMPORT_TYPE(typename distribution_type, loc_dist_metadata)


  typedef typename base_type::mapper_type::domain_type             map_dom_t;
  typedef iterator_domain<
    distribution_type, detail::get_first<Key> >         domain_type;
  typedef map_view<unordered_map>                       view_type;


  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an unordered map container.
  /// @param hash The hashing function used for distribution.
  /// @param comp The key used for associating elements
  //////////////////////////////////////////////////////////////////////
  unordered_map(hasher const& hash = hasher(),
                  key_equal const& comp = key_equal())
    : base_type(
        directory_type(
          manager_type(
            partition_type(), mapper_type(map_dom_t(get_num_locations()))
          )
        ),
        container_manager_type(partition_type(),
          mapper_type(map_dom_t(get_num_locations())), hash,comp
        )
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an unordered map container given a
  /// partition.
  /// @param partitioner The partition.
  /// @param hash The hashing function used for distribution.
  /// @param comp The key used for associating elements.
  //////////////////////////////////////////////////////////////////////
  unordered_map(partition_type const& part, hasher const& hash = hasher(),
                  key_equal const& comp = key_equal())
    : base_type(
        directory_type(
          part, mapper_type(part.domain()),
          manager_type(part, mapper_type(part.domain()))
        ),
        container_manager_type(
          part, mapper_type(map_dom_t(get_num_locations())), hash, comp
        )
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an unordered map container given a partition
  /// strategy and a mapper for the distribution.
  /// @param partitioner The partition.
  /// @param mapper The mapper used for the distribution of elements.
  /// @param hash The hashing function used for distribution.
  /// @param comp The key used for associating elements
  //////////////////////////////////////////////////////////////////////
  unordered_map(partition_type const& partitioner, mapper_type const& mapper,
                hasher const& hash = hasher(),
                key_equal const& comp = key_equal())
    : base_type(
        directory_type(partitioner, mapper, manager_type(partitioner, mapper)),
        container_manager_type(
          partitioner, mapper_type(map_dom_t(get_num_locations())), hash, comp
        )
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an unordered map with a distribution that is specified by
  /// the @p dist_view provided.
  /// @param dist_view view that specifies the mapping of container GIDs
  /// to partition ids, and partition ids to location ids.
  ///
  /// @ref distribution_spec contains a set of functions to create views
  /// for common data distributions.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  unordered_map(DistSpecView const& dist_view,
      typename std::enable_if<
        is_distribution_view<DistSpecView>::value &&
        !detail::has_is_composed_dist_spec<DistSpecView>::value
      >::type* = 0)
    : base_type(std::shared_ptr<DistSpecView>(new DistSpecView(dist_view)),
                boost::mpl::false_())
  { }

  unordered_map(unordered_map const& other)
    : base_type(other)
  { }

  /// @}

  /// @name Element Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to insert a <Key,Mapped> pair in the container.
  /// @param val The pair of <key_type,mapped_type> to be inserted.
  /// @todo: Verify if this function should return a value with
  ///   future_exist<bool> or similar.
  //////////////////////////////////////////////////////////////////////
  void insert(value_type const& val)
  {
    this->incr_version();
    this->distribution().insert(val);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to insert a Key and its mapped data in the container.
  /// @param key The Key of type key_type to be inserted.
  /// @param val The Mapped element of type mapped_type to be inserted.
  /// @todo: Verify if this function should return a value with
  ///   future_exist<bool> or similar.
  //////////////////////////////////////////////////////////////////////
  void insert(key_type const& key, mapped_type const& val)
  {
    this->incr_version();
    this->distribution().insert(value_type(key,val));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Asynchronous insert that executes the functor provided on the
  ///   value type if the element with the specified key exists, and
  ///   otherwise inserts the key-value pair provided.
  /// @param val The key-value pair inserted if the key is not found
  ///   in the container.
  /// @param f The Functor applied to val if it is present.
  //////////////////////////////////////////////////////////////////////
  template <class Functor>
  void insert(value_type const& val, Functor const& f)
  {
    this->incr_version();
    this->distribution().insert_fn(val,
                                   boost::protect(boost::bind(f, _1, val)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to modify the value of an existing element. An element
  ///   with an equivalent key should already be in the container.
  /// @param key The Key of type key_type  of the element to be modified.
  /// @param val The Mapped element of type mapped_type that will replace
  ///   the previous one.
  //////////////////////////////////////////////////////////////////////
  void set_element(key_type const& key, mapped_type const& val)
  {
    this->distribution().set_element(value_type(key,val));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to modify the value of an existing element. An element
  ///   with an equivalent key should already be in the container.
  /// @param val The element to be modified.
  //////////////////////////////////////////////////////////////////////
  void set_element(value_type const& val)
  {
    this->distribution().set_element(val);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove an element from the container asynchronously.
  /// @param k The Key of the element to be removed.
  //////////////////////////////////////////////////////////////////////
  size_t erase(key_type const& k)
  {
    this->incr_version();
    return this->distribution().erase(k);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove an element from the container.
  /// @param k The Key of the element to be removed.
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
  void clear(void)
  {
    this->incr_version();
    this->distribution().clear();
  }

  /// @}

  /// @name Element Access
  /// @{

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
  /// @brief Return an iterator over a key.
  /// @param key The Key of type key_type on which the iterator should point at
  /// @return the Iterator over the key
  /// @todo  specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  iterator find(key_type const& k)
  {
    return this->distribution().find(k);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator over a key.
  /// @param key The Key of type key_type on which the iterator should point at
  /// @return the Iterator over the key
  /// @todo  specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  const_iterator find(key_type const& k) const
  {
    return this->distribution().find(k);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Operator [ ] overloading. Used to access the Mapped element
  ///   from a specific key.
  /// @param key The key of the element to access.
  /// @return a proxy over the mapped value
  //////////////////////////////////////////////////////////////////////
  second_reference operator[](key_type const& key)
  {
    this->distribution().create(key);
    return (this->distribution().make_reference(key)).second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Used to access a  pair of <Key,Mapped> from a specific key.
  /// @param key The key of the element to access.
  /// @return A pair proxy. first is the key, second is the value
  //////////////////////////////////////////////////////////////////////
  reference make_reference(key_type const& key)
  {
    return this->distribution().make_reference(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator over a key.
  /// @param key The Key of type key_type on which the iterator
  //    should point at.
  /// @return the Iterator over the key
  /// @todo  specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(key_type const& key)
  {
    return this->distribution().make_iterator(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator over a key.
  /// @param key The Key of type key_type on which the iterator
  //    should point at.
  /// @return the Iterator over the key
  /// @todo  specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  const_iterator make_const_iterator(key_type const& key)
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
  ///   to @p key.
  /// @param key The target key
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
    else
    {
      return view_type(*this, domain_type());
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to the data (i.e., the pair's second
  ///   element) associated with a given GID.
  /// @param gid The GID associated with the element for which we want to
  ///   apply the functor and read the result.
  /// @param f The functor to apply to gid
  /// @warning This assumes that the type Functor reflects a public type
  ///   named result_type and that the invocation of its function operator
  ///   returns a value that is convertible to result_type. In addition,
  ///   Functor's function operator must be const.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void data_apply_async(gid_type const& gid, F const& f)
  {
    this->distribution().data_apply_async(gid, f);
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
  size_t size(void)
  {
    return this->distribution().size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return if the container is empty or not.
  /// @return a bool set to true if it is empty, false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool empty(void)
  {
    return (this->size()==0);
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
}; // unordered_map

} // namespace stapl

#endif
