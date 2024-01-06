/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_LIST_NULL_REGISTRY_HPP
#define STAPL_CONTAINERS_LIST_NULL_REGISTRY_HPP

#include <boost/iterator/iterator_facade.hpp>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a registry that uses the information stored in the
///        GID to determine the location where the element is stored,
///        without using any additional state information.
/// @ingroup plistDist
//////////////////////////////////////////////////////////////////////
template<typename GID>
class null_registry
{
public:
  typedef GID           key_type;
  typedef location_type value_type;

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
      : m_data(key_type(), invalid_location_id)
    { }

    const_iterator(key_type const& key, value_type const& value)
      : m_data(key, value)
    { }

    bool equal(const_iterator const& other) const
    { return (m_data==other.m_data); }

    void increment(void)
    { m_data = std::make_pair(key_type(), invalid_location_id); }

    std::pair<key_type, value_type> dereference(void) const
    { return m_data; }
  };


  template<typename T>
  explicit null_registry(T const&)
  { }

  value_type operator[](key_type const& key) const
  {
    return key.m_location;
  }

  const_iterator find(key_type const& key) const
  {
    return const_iterator(key, key.m_location);
  }

  const_iterator end(void) const
  {
    return const_iterator();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method required by @ref directory::reset().
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  { }
};

} // namespace stapl

#endif /* STAPL_CONTAINERS_LIST_NULL_REGISTRY_HPP */
