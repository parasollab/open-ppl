/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_INTERVAL_REGISTRY_HPP
#define STAPL_CONTAINERS_INTERVAL_REGISTRY_HPP

#include <boost/icl/interval_map.hpp>
#include <stapl/containers/distribution/directory/registry.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>
#include <boost/icl/type_traits/is_discrete.hpp>

namespace stapl {

/////////////////////////////////////////////////////////////////////
/// @brief @copybrief directory_registry
///
/// For this registry, GIDs are explicitly stored in aggregates through
/// the use of an interval_map from the Boost ICL library. That is,
/// if the registry logically stores the GIDs {0, 1, 2, 5, 6, 7},
/// they will be aggregated into the ranges {[0..2], [5..7]}. This
/// requires that the GIDs can be properly aggregated into intervals.
///
/// @tparam Home The manager function object
/// @tparam The less than comparator used internally in the ICL map.
/// @bug Currently, only std::less is used for the map, and the Compare
/// template parameter is ignored.
/////////////////////////////////////////////////////////////////////
template<typename Home, typename Compare = std::less<typename Home::gid_type> >
class interval_registry
{
public:
  /// GID type
  typedef typename Home::gid_type          key_type;
  /// Location type
  typedef typename Home::value_type        value_type;

private:
  /// Type of the ICL container
  typedef boost::icl::interval_map<
            key_type,
            value_type,
            boost::icl::partial_enricher
          >                                storage_type;

public:
  typedef typename storage_type::const_iterator const_iterator;

private:
  /// Interval type used by the ICL map
  typedef typename storage_type::key_type  interval_t;

  /// Manager function object
  Home           m_home;

  /// Map storing intervals
  storage_type   m_registry;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a registry with a stateless home functor
  //////////////////////////////////////////////////////////////////////
  interval_registry(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a registry with a given home function
  /// @param home The manager function object
  //////////////////////////////////////////////////////////////////////
  explicit interval_registry(Home const& home)
    : m_home(home)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Resets the partition and mapper stored in m_home.
  ///
  /// @param partition view-based partition of a data distribution.
  /// @param mapper view-based mapper of a data distribution.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition, typename Mapper>
  void reset_home(Partition const& partition, Mapper const& mapper)
  {
    // data has been redistributed in to partitions.  At this point there
    // are no exceptions.
    m_registry.clear();
    m_home.partition(partition);
    m_home.mapper(mapper);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear all of the currently mapped GIDs in the registry
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    m_registry.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return whether or not this registry has any entries
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return m_registry.empty();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc find
  //////////////////////////////////////////////////////////////////////
  value_type operator[](key_type const& key) const
  {
    const_iterator it = m_registry.find(key);
    if (it != m_registry.end())
      return it->second;
    else
      return m_home(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Register the locality of a single GID.
  /// @param val Key-value pair to insert into the registry
  //////////////////////////////////////////////////////////////////////
  void insert(std::pair<key_type, value_type> const& val)
  {
    interval_t range =
      boost::icl::construct<interval_t>(val.first, val.first,
        boost::icl::interval_bounds::closed()
      );

    m_registry.set(std::make_pair(range,val.second));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Register the locality of a range of GIDs.
  /// @param val Key-value pair where the first is an interval of GIDs and
  /// the second is the location
  //////////////////////////////////////////////////////////////////////
  template <typename Keys>
  void insert(std::pair<Keys, value_type> const& val)
  {
    interval_t range =
      boost::icl::construct<interval_t>(val.first.first, val.first.second,
        boost::icl::interval_bounds::closed()
      );

    m_registry.set(std::make_pair(range,val.second));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Lookup in the registry on which location a GID resides.
  /// @param key Key to look up in the registry
  /// @return An iterator that when dereferenced returns a std::pair with
  ///         the key and the location of the key.
  /// @todo This function is unnecessarily complex, but @ref erase() will place
  ///       an invalid entry, not remove the registered key.
  //////////////////////////////////////////////////////////////////////
  const_iterator find(key_type const& key) const
  {
    const_iterator it = m_registry.find(key);

    if (it==m_registry.end() || it->second==index_bounds<value_type>::invalid())
      return m_registry.end();

    return it;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erase a mapping of a GID from a location.
  /// @param key Key to remove from the directory
  /// @todo The key should be deleted, not replaced with invalid.
  //////////////////////////////////////////////////////////////////////
  void erase(key_type const& key)
  {
    insert(std::make_pair(key, index_bounds<value_type>::invalid()));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a value representing the end of the registry.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return m_registry.end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether or not a given key is mapped in the registry
  /// @param key The key to query
  /// @return True if any mapping of this key is in the directory
  /// @todo Should this return true if the key is currently mapped to
  /// an invalid location?
  //////////////////////////////////////////////////////////////////////
  bool contains(key_type const& key) const
  {
    return (m_registry.find(key)!=m_registry.end());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the first element in the interval to right of that
  ///   containing @p key as well as the location @p key is mapped to.
  ///
  /// Used for insertion in dynamic containers.
  //////////////////////////////////////////////////////////////////////
  std::pair<key_type, location_type>
  shift_right(key_type const& key)
  {
    auto iter = m_registry.find(key);

    stapl_assert(iter != m_registry.end(), "could not find key");

    const key_type last          = boost::icl::last(iter->first);
    const location_type location = iter->second;

    // else
    return std::make_pair(last+1, location);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove the last key in the interval containing @p key.  Return
  ///   the last key in the adjacent interval to the left together with the
  ///   location this interval maps to, so that last gid can be updated.
  ///
  /// Used for erasure in dynamic containers.
  //////////////////////////////////////////////////////////////////////
  std::pair<key_type, location_type>
  shift_left(key_type const& key, key_type const& target_key)
  {
    auto iter = m_registry.find(key);

    stapl_assert(iter != m_registry.end(), "could not find key");

    const key_type first         = boost::icl::first(iter->first);
    const key_type last          = boost::icl::last(iter->first);
    const location_type location = iter->second;

    const bool b_contains_target =
      boost::icl::contains(iter->first, target_key);

    m_registry.erase(last);

    if (b_contains_target)
      return std::make_pair(first, invalid_location_id);

    // else
    return std::make_pair(first-1, location);
  }
}; // class interval_registry

} // namespace stapl

#endif // STAPL_CONTAINERS_INTERVAL_REGISTRY_HPP
