/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_STATIC_REGISTRY_HPP
#define STAPL_CONTAINERS_STATIC_REGISTRY_HPP

#include <stapl/containers/type_traits/index_bounds.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <utility>

namespace stapl {

/////////////////////////////////////////////////////////////////////
/// @brief This class is used to store directory entries in the
/// container_directory class and subsequently, its base class directory. It
/// models the map concept.
///
/// For static pContainers, this registry will simply mimic the initial
/// distribution of GIDs, which is provided by the @p Home function object.
///
/// @tparam Home Function object that provides a (usually) closed-form solution
/// for GID -> location.
//////////////////////////////////////////////////////////////////////
template<typename Home>
class static_registry
{
public:
  typedef typename Home::gid_type   key_type;
  typedef typename Home::value_type value_type;

  class const_iterator
    : public boost::iterator_facade<
               const_iterator,
               std::pair<key_type, value_type> const,
               boost::incrementable_traversal_tag,
               std::pair<key_type, value_type>
             >
  {
  private:
    std::pair<key_type, value_type> m_data;

    friend class boost::iterator_core_access;

  public:
    const_iterator(void)
      : m_data(key_type(), index_bounds<value_type>::invalid())
    { }

    const_iterator(key_type const& key, value_type const& value)
      : m_data(key, value)
    { }

    bool equal(const_iterator const& other) const
    { return (m_data==other.m_data); }

    void increment(void)
    {
      m_data = std::make_pair(key_type(), index_bounds<value_type>::invalid());
    }

    std::pair<key_type, value_type> dereference(void) const
    { return m_data; }
  };

private:
  /// The manager
  Home m_home;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create a registry with a given manager (home).
  /// @param home Function object to translate keys to values.
  //////////////////////////////////////////////////////////////////////
  explicit static_registry(Home const& home)
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
    m_home.partition(partition);
    m_home.mapper(mapper);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the storage of the registry. As there is no storage,
  /// this is effectively a no op.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether or not the registry is empty. The registry
  /// is only empty if there is no data defined by the container.
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
     return m_home.partition().global_domain().empty();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc find
  //////////////////////////////////////////////////////////////////////
  value_type operator[](key_type const& key) const
  {
    return m_home(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Register the locality of a GID. This is not valid for a static
  /// registry.
  /// @param val Key-value pair to insert into the registry
  //////////////////////////////////////////////////////////////////////
  void insert(std::pair<key_type, value_type> const&)
  {
    stapl_assert(false, "attempting to insert into a static registry");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Lookup in the registry on which location a GID resides.
  /// @param key Key to look up in the registry.
  /// @return The location of the key.
  //////////////////////////////////////////////////////////////////////
  const_iterator find(key_type const& key) const
  {
    const std::pair<value_type, loc_qual> v = m_home(key);
    stapl_assert(v.second != LQ_LOOKUP,
      "static_registry::find instructed to forward request.");
    return (v.first !=index_bounds<value_type>::invalid() ?
            const_iterator(key, v.first ) : const_iterator());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erase a mapping of a GID from a location. Erasing from a static
  /// registry is not valid.
  /// @param key Key to remove from the directory.
  //////////////////////////////////////////////////////////////////////
  void erase(key_type const&)
  {
    stapl_assert(false, "attempting to erase from a static registry");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a value representing the end of the registry.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return const_iterator();
  }
};

}

#endif // STAPL_CONTAINERS_STATIC_REGISTRY_HPP
