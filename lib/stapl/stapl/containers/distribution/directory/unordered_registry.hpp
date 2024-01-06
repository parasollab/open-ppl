/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_REGISTRY_HPP
#define STAPL_CONTAINERS_UNORDERED_REGISTRY_HPP

#include <stapl/containers/distribution/directory/registry.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>

namespace stapl {

/////////////////////////////////////////////////////////////////////
/// @copybrief directory_registry
///
/// For this registry, GIDs are mapped to their locations using an
/// unordered map.
///
/// @tparam Home The manager function object
/// @tparam The less than comparator used internally in the unordered map.
///
/// @todo Modify the class to use the pContainer's hash function to determine
/// the location of an element rather than explicitly storing it.
/////////////////////////////////////////////////////////////////////
template<typename Home, typename Compare = std::less<typename Home::gid_type> >
class unordered_registry
{
public:
  /// GID type
  typedef typename Home::gid_type          key_type;
  /// Location type
  typedef typename Home::value_type        value_type;

private:
  /// Type of the container
  typedef boost::unordered_map<
            key_type,
            value_type,
            stapl::hash<key_type>
          >                                storage_type;

public:
  typedef typename storage_type::const_iterator const_iterator;

private:
  /// Manager function object
  Home           m_home;
  /// Map storing intervals
  storage_type   m_registry;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a registry with a stateless home functor
  //////////////////////////////////////////////////////////////////////
  unordered_registry(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a registry with a given home function
  /// @param home The manager function object
  //////////////////////////////////////////////////////////////////////
  explicit unordered_registry(Home const& home)
    : m_home(home)
  { }

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
  /// @brief Lookup in the registry on which location a GID resides.
  /// @param key Key to look up in the registry
  /// @return The location where the key resides.
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
    m_registry.insert(val);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Lookup in the registry on which location a GID resides.
  /// @param key Key to look up in the registry
  /// @return An iterator that when dereferenced returns a @ref std::pair with
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
    m_registry.erase(key);
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
  /// @return True if a valid mapping of this key is in the directory
  //////////////////////////////////////////////////////////////////////
  bool contains(key_type const& key) const
  {
    const_iterator it = m_registry.find(key);
    if (it==m_registry.end() || it->second==index_bounds<value_type>::invalid())
      return false;
    return true;
  }
}; // class unordered_registry

} // stapl namespace

#endif /* STAPL_CONTAINERS_UNORDERED_REGISTRY_HPP */
