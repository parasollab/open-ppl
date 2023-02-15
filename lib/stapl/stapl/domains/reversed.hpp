/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_REVERSED_HPP
#define STAPL_DOMAINS_REVERSED_HPP

#include <stapl/containers/type_traits/index_bounds.hpp>
#include <cstddef>
#include <iosfwd>

#include <iostream>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Represents a one dimensional reversed domain over the given
///        domain @p Dom.
///
/// This domain is used to adapt a subdomain to represent the
/// correct reversed set of indexes and is intended to be used
/// only during the domain's projection in the coarsening process.
/// @todo: compute the non-empty size
/// @todo: determine whether domain empty
//////////////////////////////////////////////////////////////////////
template <typename Dom>
class reversed_domain
  : public Dom
{
public:
  typedef typename Dom::index_type   index_type;
  typedef index_type                 gid_type;
  typedef std::size_t                size_type;

private:
  size_type m_total_size;
  long      m_factor;

  Dom calc_dom(index_type const& f,
               index_type const& l,
               reversed_domain const& o)
  {
    Dom tmp_dom(f,l,o);
    Dom tmp_o(o);
    size_type sz = o.distance(o.first(), f);
    long factor = 2*(tmp_o.last()+1-sz) - tmp_dom.size() - o.m_total_size;
    return Dom(f+factor, l+factor, o);
  }

public:
  reversed_domain(void)
    : m_total_size(0),
      m_factor(0)
  { }

  reversed_domain(Dom const& dom)
    : Dom(dom),
      m_total_size(dom.size()),
      m_factor(Dom::size() + m_total_size - 2*(Dom::last()+1))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a domain based on the specified domain @p dom
  ///        and the total number of elements @p total_size of the
  ///        domain from where this subdomain belongs to.
  //////////////////////////////////////////////////////////////////////
  reversed_domain(Dom const& dom, size_type total_size)
    : Dom(dom),
      m_total_size(total_size),
      m_factor(Dom::size() + m_total_size - 2*(Dom::last()+1))
  { }

  reversed_domain(index_type const& first,
                  index_type const& last,
                  reversed_domain const& other)
    : Dom(calc_dom(first,last,other)),
      m_total_size(other.m_total_size),
      m_factor(Dom::size() + m_total_size - 2*(Dom::last()+1))
  { }

  //////////////////////////////////////////////////////////////////////
  // common functionality for all domains
  //////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::first
  //////////////////////////////////////////////////////////////////////
  index_type first(void) const
  {
    return Dom::first()+m_factor;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::last
  //////////////////////////////////////////////////////////////////////
  index_type last(void) const
  {
    return Dom::last()+m_factor;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::open_last
  //////////////////////////////////////////////////////////////////////
  index_type open_last(void) const
  {
    return Dom::open_last()+m_factor;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::contains
  //////////////////////////////////////////////////////////////////////
  bool contains(index_type const& idx) const
  {
    return Dom::contains(idx-m_factor);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::size
  //////////////////////////////////////////////////////////////////////

  // size() goes here

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::empty
  //////////////////////////////////////////////////////////////////////

  // empty() goes here

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::is_same_container_domain
  //////////////////////////////////////////////////////////////////////
  bool is_same_container_domain(void) const
  {
    return Dom::is_same_container_domain();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "REVERSED_DOMAIN %p: " << this;
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << "\n";
    std::cerr << " first " << first();
    std::cerr << " last " << last() << "\n";;
    //std::cerr << " size " << size() << "\n";
  }
};

} // namespace stapl

#endif /* STAPL_DOMAINS_REVERSED_HPP */
