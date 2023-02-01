/*
// Copyright (c) 2000-2011, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_DOMAINS_EXPLICIT_DOMAIN_HPP
#define STAPL_DOMAINS_EXPLICIT_DOMAIN_HPP

#include <stapl/runtime.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/views/type_traits/is_domain_sparse.hpp>
#include <algorithm>
#include <iterator>
#include <iosfwd>
#include <vector>
#include <boost/mpl/bool.hpp>

#include <iostream>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines an explicit one dimensional domain composed of indexes.
/// The order of the indexes in the domain is given by the user.
/// Each index that is valid in this domain must be explicitly enumerated.
/// @tparam T Index type.
//////////////////////////////////////////////////////////////////////
template<typename T>
class explicit_domain
{
public:
  typedef T              gid_type;
  typedef gid_type       index_type;
  typedef std::size_t    size_type;
  typedef std::vector<T> sequence_type;

private:
  sequence_type m_domain;
  bool          m_cont_dom;

public:
  explicit_domain(void)
    : m_cont_dom(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the domain with the elements of a std vector.
  /// The order of the elements is maintained.
  /// @param sequence sequence of indexes.
  /// @param is_cont_dom true if represents the entire domain of a pContainer.
  //////////////////////////////////////////////////////////////////////
  explicit_domain(sequence_type const& sequence, bool is_cont_dom = false)
    : m_domain(sequence),
      m_cont_dom(is_cont_dom)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new domain by restricting the given @p other
  ///        domain to be [lower..upper].
  //////////////////////////////////////////////////////////////////////
  explicit_domain(index_type const& lower,
                  index_type const& upper,
                  explicit_domain const& other)
    : m_cont_dom(false)
  {
    typename sequence_type::iterator first_it, last_it;
    first_it = std::find(other.m_cont_dom.begin(),
                         other.m_cont_dom.end(), lower);
    last_it = std::find(other.m_cont_dom.begin(),
                        other.m_cont_dom.end(), upper);
    stapl_assert(first_it !=  other.m_cont_dom.end(),
                 "error: explicit_domain constructor");
    stapl_assert(last_it !=  other.m_cont_dom.end(),
                 "error: explicit_domain constructor");
    m_domain = sequence_type(first_it, last_it+1);
  }

  //////////////////////////////////////////////////////////////////////
  // common functionality for all domains
  //////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::first
  //////////////////////////////////////////////////////////////////////
  index_type first(void) const
  {
    return m_domain.front();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::last
  //////////////////////////////////////////////////////////////////////
  index_type last(void) const
  {
    return m_domain.back();
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
  bool contains(index_type const& idx) const
  {
    return (find(m_domain.begin(), m_domain.end(), idx) != m_domain.end());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::size
  //////////////////////////////////////////////////////////////////////
  size_type size(void) const
  {
    return m_domain.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::empty
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return m_domain.empty();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::is_same_container_domain
  //////////////////////////////////////////////////////////////////////
  bool is_same_container_domain(void) const
  {
    return m_cont_dom;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "EXPLICIT_DOMAIN %p: " << this;
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
  /// @copydoc stapl::indexed_domain::advance(index_type, D)
  //////////////////////////////////////////////////////////////////////
  template<typename Distance>
  index_type advance(index_type const& idx, Distance n) const
  {
    typename sequence_type::const_iterator index_it;
    index_it = find(m_domain.begin(), m_domain.end(), idx);

    if (std::distance(index_it+n, m_domain.end()) <= 0)
      return index_bounds<gid_type>::invalid();

    return *(index_it + n);
  }

  size_type distance(index_type const& i0, index_type const& i1) const
  {
    typename sequence_type::iterator
    first_it = std::find(m_domain.begin(), m_domain.end(), i0);
    typename sequence_type::iterator
    last_it = std::find(m_domain.begin(), m_domain.end(), i1);
    auto dist = std::distance(first_it, last_it);
    return (dist < 0) ? -dist : dist;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the intersection between this domain and the
  ///        given @p other domain.
  //////////////////////////////////////////////////////////////////////
  explicit_domain operator&(explicit_domain const& other) const
  {
    sequence_type result;
    sequence_type other_copy(other);
    typename sequence_type::const_iterator it;
    std::sort(other_copy.begin(), other_copy.end());
    for (it = m_domain.begin(); it != m_domain.end(); ++it)
    {
      if (std::binary_search(other_copy.begin(), other_copy.end(), *it))
        result.push_back(*it);
    }
    return explicit_domain(result);
  }

  sequence_type const& get_sequence(void) const
  {
    return m_domain;
  }

  void define_type(typer &t)
  {
    t.member(m_domain);
    t.member(m_cont_dom);
  }
};


template<typename T>
std::ostream& operator<<(std::ostream &os, explicit_domain<T> const& d)
{
  os << "[";
  for (typename explicit_domain<T>::size_type i=0; i<d.size(); ++i) {
    os << d.advance(d.first(),i);
    os << ", ";
  }
  return os << "]";
}


template <typename T>
struct is_domain_sparse<explicit_domain<T> >
  : boost::mpl::true_
{ };

} // namespace stapl

#endif
