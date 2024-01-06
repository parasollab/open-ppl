/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_CONTINUOUS_DOMAIN_HPP
#define STAPL_DOMAINS_CONTINUOUS_DOMAIN_HPP

#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/runtime/serialization_fwd.hpp>
#include <cstddef>
#include <iosfwd>

#include <iostream>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a domain where the indexes are continuous and not
///        numerable (e.g., float, string), but there is an order
///        relation among the indexes (e.g., lexicographical order).
///
/// The domain is defined as a range of indexes between first index
/// and last index (included) @$ [first..last]@$.
///
/// @tparam T Index type.
//////////////////////////////////////////////////////////////////////
template<typename T>
class continuous_domain
{
public:
  typedef T           index_type;
  typedef index_type  gid_type;
  typedef std::size_t size_type;

private:
  index_type m_first;
  index_type m_last;
  /// @brief @c true if the domain references all the elements in the container.
  bool       m_full_container;

public:
  continuous_domain(void)
    : m_first(index_bounds<index_type>::invalid()),
      m_last(index_bounds<index_type>::invalid()),
      m_full_container(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a domain containing the indexes from @p first
  ///        to @p last (included).
  ///
  /// If @p full_container is specified as @c true, the domain will be
  /// flagged as a domain that contains all the indexes of a container.
  //////////////////////////////////////////////////////////////////////
  continuous_domain(index_type const& first,
                    index_type const& last,
                    bool full_container = false)
    : m_first(first),
      m_last(last),
      m_full_container(full_container)
  { }

  //////////////////////////////////////////////////////////////////////
  // common functionality for all domains
  //////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::first
  //////////////////////////////////////////////////////////////////////
  index_type first(void) const
  {
    return m_first;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::last
  //////////////////////////////////////////////////////////////////////
  index_type last(void) const
  {
    return m_last;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::open_last
  //////////////////////////////////////////////////////////////////////
  index_type open_last(void) const
  {
    return index_bounds<index_type>::invalid();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::contains
  //////////////////////////////////////////////////////////////////////
  bool contains(index_type const& g) const
  {
    return (g == m_first) || (g == m_last) || ((m_first < g) && (g < m_last));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::size
  //////////////////////////////////////////////////////////////////////
  size_type size(void) const
  {
    return std::numeric_limits<size_type>::max();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::empty
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return ((m_first == index_bounds<index_type>::invalid()) &&
            (m_last == index_bounds<index_type>::invalid()));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::is_same_container_domain
  //////////////////////////////////////////////////////////////////////
  bool is_same_container_domain(void) const
  {
    return m_full_container;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "CONTINUOUS_DOMAIN %p: " << this;
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << "\n";
    std::cerr << " first " << first();
    std::cerr << " last " << last();
    std::cerr << " size " << size() << "\n";
  }

  //////////////////////////////////////////////////////////////////////
  // unique functionality for this domain
  //////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns how many positions are needed to advance from the
  ///        index @p i to reach the index @p j.
  /// @note The returned value does not define in which direction
  ///       advance.
  //////////////////////////////////////////////////////////////////////
  size_type distance(index_type const&, index_type const&) const
  {
    return std::numeric_limits<size_type>::max();
  }

  bool less_than(index_type const& i0, index_type const& i1) const
  {
    return (i0<i1);
  }

  void define_type(typer& t)
  {
    t.member(m_first);
    t.member(m_last);
    t.member(m_full_container);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the resulting intersection with the given @p other
  ///        domain.
  //////////////////////////////////////////////////////////////////////
  template <typename ODom>
  continuous_domain operator&(ODom const& other) const
  {
    if (!m_full_container) {
      typename ODom::index_type olower = other.first();
      typename ODom::index_type oupper = other.last();

      if (!empty()) {
        if (other.contains(m_first)) {
          olower = m_first;
        }
        if (other.contains(m_last)) {
          oupper = m_last;
        }
      }
      if (!(other.contains(m_first) || other.contains(m_last)) )
        if (!(contains(olower) || contains(oupper)))
          return continuous_domain();
      return continuous_domain(olower,oupper);
    }
    else
      return continuous_domain(other.first(), other.last());
  }
};


template<typename T>
std::ostream& operator<<(std::ostream &os, continuous_domain<T> const& d)
{
  if (d.empty()) {
    return os << "empty";
  }
  return os << "[" << d.first() << ".." << d.last() << "]";
}

} // namespace stapl

#endif // STAPL_DOMAINS_CONTINUOUS_DOMAIN_HPP
