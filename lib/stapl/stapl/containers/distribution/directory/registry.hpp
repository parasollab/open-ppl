/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DIRECTORY_REGISTRY_HPP
#define STAPL_CONTAINERS_DIRECTORY_REGISTRY_HPP

#include <stapl/utility/hash.hpp>
#include <stapl/utility/down_cast.hpp>
#include <boost/unordered_map.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <utility>

namespace stapl {

namespace detail {


/////////////////////////////////////////////////////////////////////
/// @brief Abstract base class, a pointer of which is held in a registry
/// object, allowing it to interact with a home manager object without
/// knowing its concrete type, just it's gid / key type.
/////////////////////////////////////////////////////////////////////
template<typename GID>
struct home_storage_base
{
  virtual ~home_storage_base() = default;

  virtual std::pair<location_type, loc_qual> lookup(GID const&) = 0;

  /////////////////////////////////////////////////////////////////////
  /// @brief Used to copy construct a new instance of the most derived object.
  /////////////////////////////////////////////////////////////////////
  virtual std::unique_ptr<home_storage_base> clone(void) const = 0;

  /////////////////////////////////////////////////////////////////////
  /// @brief Allows caller to specify mapper and partition of the home
  /// manager, so that we can cast down to @ref home_storage_intermediate to
  /// dispatch a virtual call to the reset methods of the manager for each
  /// of these components.
  /////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  void reset(Partition const& partition, Mapper const& mapper);
};


/////////////////////////////////////////////////////////////////////
/// @brief Intermediate base class of @ref home_storage which encodes
/// the type of the partition and mapper.  Allows us to support
/// quasi-virtual member template call.
/////////////////////////////////////////////////////////////////////
template<typename GID, typename Partition, typename Mapper>
struct home_storage_intermediate
  : home_storage_base<GID>
{
  virtual void reset_impl(Partition const& partition, Mapper const& mapper) = 0;
};


template<typename GID>
template<typename Partition, typename Mapper>
void
home_storage_base<GID>::reset(Partition const& partition, Mapper const& mapper)
{
  using derived_t = home_storage_intermediate<GID, Partition, Mapper>;

  down_cast<derived_t&>(*this).reset_impl(partition, mapper);
}


/////////////////////////////////////////////////////////////////////
/// @brief Derived class instantiated by registry to hold an instance
/// of the manager.  Implements virtual methods for lookup and partition
/// & mapper resets.
/////////////////////////////////////////////////////////////////////
template<typename Home>
struct home_storage final
  : home_storage_intermediate<
      typename Home::gid_type,
      typename Home::partition_type,
      typename Home::mapper_type
    >
{
  Home m_home;

  using partition_type = typename Home::partition_type;
  using mapper_type    = typename Home::mapper_type;
  using gid_type       = typename Home::gid_type;

  home_storage(Home const& home)
    : m_home(home)
  { }

  void reset_impl(partition_type const& partition,
                  mapper_type const& mapper) override
  {
    m_home.partition(partition);
    m_home.mapper(mapper);
  }

  std::pair<location_type, loc_qual> lookup(gid_type const& gid) override
  {
    return m_home(gid);
  }

 std::unique_ptr<home_storage_base<gid_type>> clone(void) const override
 {
   return std::unique_ptr<home_storage_base<gid_type>>
     (new home_storage(m_home));
 }
}; // struct home_storage


/////////////////////////////////////////////////////////////////////
/// @brief Custom iterator for @ref directory_registry, returned to callers
/// of @p find method.
///
/// @todo See if iteration is actually used or if we just compare to end.
/// If not just return a pointer which we can compare to nullptr.
/////////////////////////////////////////////////////////////////////
template<typename Key>
class directory_registry_iterator
  : public boost::iterator_facade<
             directory_registry_iterator<Key>,
             std::pair<Key, location_type> const,
             boost::incrementable_traversal_tag,
             std::pair<Key, location_type>>
{
private:
  using value_type = location_type;

  std::pair<Key, value_type> m_data;

  friend class boost::iterator_core_access;

public:
  directory_registry_iterator(void)
    : m_data(Key(), index_bounds<value_type>::invalid())
  { }

  directory_registry_iterator(Key const& key, value_type const& value)
    : m_data(key, value)
  { }

  bool equal(directory_registry_iterator const& other) const
  { return (m_data == other.m_data); }

  void increment(void)
  {
    m_data = std::make_pair(Key(), index_bounds<value_type>::invalid());
  }

  std::pair<Key, value_type> dereference(void) const
  { return m_data; }
}; // class directory_registry_iterator

} // namespace detail

/////////////////////////////////////////////////////////////////////
/// @brief This class is used to store directory entries in the @ref
/// container_directory class and subsequently, its base class directory. It
/// models the map concept. For most pContainers, it is a safe assumption to
/// expect that most GIDs will be mapped to the home location and that there
/// will be only a small amount of GIDs that differ from this closed-form
/// solution (through migration, etc.)
///
/// In these cases (i.e., for this class), the map only explicitly stores
/// values that differ from the closed-form home calculation.
///
/// @todo For the use case of this registry in directory, it may be pertinent
/// to actually use some sort of fake iterators for operations that would make
/// this container more STL friendly.
/// @tparam Home Function object that provides a (usually) closed-form solution
/// for key -> value.
///
/// @see container_directory
//////////////////////////////////////////////////////////////////////
template<typename Key>
class directory_registry
{
public:
  using key_type       = Key;
  using value_type     = location_type;
  using const_iterator = detail::directory_registry_iterator<Key>;

private:
  using pair_type     = std::pair<value_type, loc_qual>;
  using storage_type  =
    boost::unordered_map<key_type, location_type, stapl::hash<key_type>>;

  /// @brief The manager
  std::unique_ptr<detail::home_storage_base<Key>>   m_home_ptr;

  /// @brief Stores keys that are exceptions (i.e. known to deviate from
  /// the partition/mapper derived home location defined in the manager/Home.
  storage_type                                     m_storage;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create a registry with a given manager (home).
  /// @param home Function object to translate keys to values.
  //////////////////////////////////////////////////////////////////////
  template<typename Home>
  explicit
  directory_registry(Home const& home)
    : m_home_ptr(new detail::home_storage<Home>(home))
  { }

  directory_registry(directory_registry const& other)
    : m_home_ptr(other.m_home_ptr->clone()),
      m_storage(other.m_storage)
  { }

  directory_registry& operator=(directory_registry other)
  {
    other.swap(*this);
    return *this;
  }

  directory_registry& swap(directory_registry& other)
  {
    using std::swap;
    swap(m_home_ptr, other.m_home_ptr);
    swap(m_storage, other.m_storage);
    return *this;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Resets the partition and mapper stored in m_home.
  ///
  /// @param partition view-based partition of a data distribution.
  /// @param mapper view-based mapper of a data distribution.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition, typename Mapper>
  void reset_home(Partition const& partition, Mapper const& mapper)
  {
    // Data has been redistributed into partitions.
    // At this point there are no exceptions.
    m_storage.clear();

    m_home_ptr->reset(partition, mapper);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the storage of the registry
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    m_storage.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether or not the registry is empty.
  /// @todo Currently not implemented
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc find
  //////////////////////////////////////////////////////////////////////
  value_type operator[](key_type const& key) const
  {
    return find(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Register the locality of a GID.
  /// @param val Key-value pair to insert into the registry
  ///
  /// If the key is mapped to this location by the home manager, see if
  /// it is in the exception list (possible for a previous erase call).
  /// If so, erase exception.  If it's not mapped to this location, add
  /// an entry in the exception list with the alternative location.
  //////////////////////////////////////////////////////////////////////
  void insert(std::pair<key_type, value_type> const& val)
  {
    pair_type loc = m_home_ptr->lookup(val.first);

    stapl_assert(loc.second != LQ_LOOKUP,
      "directory_registry::insert instructed to forward request.");

    if (val.second == loc.first)
    {
      typename storage_type::iterator iter = m_storage.find(val.first);

      stapl_assert(
        iter->second == index_bounds<value_type>::invalid(),
        "found a registry exception on insert and location was valid"
      );

      if (iter != m_storage.end())
        m_storage.erase(iter);

      return;
    }

    m_storage[val.first] = val.second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Lookup in the registry on which location a GID resides.
  /// @param key Key to look up in the registry
  /// @return The location of the key
  ///
  /// Checks if the key is in the list of exceptions from the home location.
  /// If so, return the location explicitly stored there.  Otherwise, return
  /// the location the manager/home location assigns to the given GID.
  ///
  /// @todo If the @ref erase() is fixed so it removes elements from the map,
  /// then one of the ifs can go away, as if something is in the map, then it's
  /// valid.
  //////////////////////////////////////////////////////////////////////
  const_iterator find(key_type const& key) const
  {
    auto it = m_storage.find(key);

    value_type v;

    if (it == m_storage.end())
    {
      auto loc = m_home_ptr->lookup(key);
      stapl_assert(loc.second != LQ_LOOKUP,
        "registry::find instructed to forward request.");
      v = loc.first;
    }
    else
      v = it->second;

    return (v!=index_bounds<value_type>::invalid() ? const_iterator(key, v)
                                                   : const_iterator());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erase a mapping of a GID from a location.
  /// @param key Key to remove from the directory
  ///
  /// When erasing, leave a invalid entry in the exception list.  This keeps
  /// it in a transition state until a subsequent insert (for static containers,
  /// with migration) reenables it in the registry to handle messages on the
  /// new location (needed for ordering protocol, messages revert to being
  /// buffered in directory).
  ///
  /// @todo For dynamic containers where elements won't be reinserted following
  /// migration, there can be a build up of unneeded, invalid entries in the
  /// exception list.  Need to change migration not to need invalid transition
  /// state or have another method.  Bottom line, there's an expectation that
  /// erase is called by a subsequent insert to flush the stale exception entry.
  ///
  /// @todo Why not remove it from the map completely?
  //////////////////////////////////////////////////////////////////////
  void erase(key_type const& key)
  {
    m_storage[key] = index_bounds<value_type>::invalid();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a value representing the end of the registry.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return const_iterator();
  }
}; // class directory_registry

} // namespace stapl

#endif // STAPL_CONTAINERS_DIRECTORY_REGISTRY_HPP
