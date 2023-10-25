/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_INTERVAL_HPP
#define STAPL_DOMAINS_INTERVAL_HPP

#include <cstddef>
#include <iosfwd>
#include <iostream>

#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/domains/indexed.hpp>

#include <boost/icl/discrete_interval.hpp>
#include <boost/icl/interval_set.hpp>
#include <boost/mpl/bool.hpp>

namespace stapl {

template<typename Distribution>
class domainset1D;

//////////////////////////////////////////////////////////////////////
/// @brief Defines an one dimensional domain composed of a set of
///        intervals of consecutive indexes.
///
/// @tparam T Index type.
///
/// @warning @c T must be and integral type.
/// @todo Add methods to make the class iterable in order to avoid copying
/// values into a std::vector in construct_vbdist_element for construct call.
//////////////////////////////////////////////////////////////////////
template <typename T>
class domset1D
{
public:
  typedef T                                         index_type;
  typedef default_traversal<1>::type                traversal_type;
  typedef std::size_t                               size_type;
  typedef index_type                                gid_type;
  typedef boost::icl::interval_set<index_type>      set_type;

private:
  typedef boost::icl::discrete_interval<index_type> int_type;

  std::shared_ptr<set_type>                         m_set;
  bool                                              m_cont_dom;

public:
  domset1D(set_type const& s, bool is_cont_dom = false)
    : m_set(new set_type(s)),
      m_cont_dom(is_cont_dom)
  { }

  domset1D(void)
    : m_set(new set_type),
      m_cont_dom(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a domain with the given @p lower and @p upper
  ///        bounds [lower .. upper].
  /// @param lower Lower index in the domain.
  /// @param upper Upper index in the domain.
  /// @param is_cont_dom whether or not this domain spans the entire container.
  //////////////////////////////////////////////////////////////////////
  domset1D(index_type const& lower, index_type const& upper,
    bool is_cont_dom = false)
    : m_set(new set_type(int_type::closed(lower, upper))),
      m_cont_dom(is_cont_dom)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::indexed_domain(size_type)
  //////////////////////////////////////////////////////////////////////
  explicit domset1D(size_type size, bool is_cont_dom = true)
    : m_set(new set_type(int_type::closed(0, size-1))),
      m_cont_dom(is_cont_dom)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new domain by restricting the given @p other
  ///        domain to be [first..last].
  //////////////////////////////////////////////////////////////////////
  domset1D(T const& lower, T const& upper, domset1D const& other)
    : m_set(new set_type(int_type::closed(lower, upper) & *other.m_set)),
      m_cont_dom(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new domain by converting it from an existing
  ///        @ref indexed_domain.
  //////////////////////////////////////////////////////////////////////
  domset1D(indexed_domain<gid_type> const& dom)
    : m_set(new set_type(int_type::closed(dom.first(), dom.last()))),
      m_cont_dom(dom.is_same_container_domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new domain by extracting it from an existing
  ///        @ref domainset1D domain.
  //////////////////////////////////////////////////////////////////////
  template<typename Dist>
  domset1D(domainset1D<Dist> const& dom)
  {
    *this = dom.get_sparse_domain();
  }

  //////////////////////////////////////////////////////////////////////
  // common functionality for all domains
  //////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::first
  //////////////////////////////////////////////////////////////////////
  index_type first(void) const
  {
    return m_set->empty() ? index_bounds<index_type>::invalid()
                          : boost::icl::first(*m_set->begin());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::last
  //////////////////////////////////////////////////////////////////////
  index_type last(void) const
  {
    return m_set->empty() ? index_bounds<index_type>::invalid()
                          : boost::icl::last(*(--m_set->end()));
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
    return boost::icl::contains(*m_set, idx);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::size
  //////////////////////////////////////////////////////////////////////
  size_type size(void) const
  {
    return m_set->size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::dimensions
  //////////////////////////////////////////////////////////////////////
  size_type dimensions(void) const
  {
    return size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::empty
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return m_set->empty();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::is_same_container_domain
  //////////////////////////////////////////////////////////////////////
  bool is_same_container_domain(void) const
  {
    return m_cont_dom;
  }

  void set_cont_dom(bool is_cont_dom = true)
  {
    m_cont_dom = is_cont_dom;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "INTERVAL_DOMAIN %p: " << this;
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << "\n";
    std::cerr << " first " << first();
    std::cerr << " last " << last();
    std::cerr << " size " << size();
    std::cerr << " cont " << m_cont_dom << "\n";
  }

  bool operator==(domset1D const& other) const
  {
    return m_cont_dom == other.m_cont_dom &&
           *m_set == *other.m_set;
  }
  //////////////////////////////////////////////////////////////////////
  // unique functionality for this domain
  //////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds all the indexes in the given @p other domain.
  ///
  /// Performs an union operation between this domain and @p other
  /// domain.
  //////////////////////////////////////////////////////////////////////
  domset1D& operator+=(domset1D const& other)
  {
    if (!m_set.unique())
      m_set.reset(new set_type(*m_set));
    *m_set += *other.m_set;
    m_cont_dom = false;
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the given @p index to the domain.
  //////////////////////////////////////////////////////////////////////
  domset1D& operator+=(index_type const& index)
  {
    if (!m_set.unique())
      m_set.reset(new set_type(*m_set));
    *m_set += index;
    m_cont_dom = false;
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all the indexes in the given @p other domain from
  ///        this domain.
  ///
  /// Performs a difference between this domain and the given @p other domain.
  //////////////////////////////////////////////////////////////////////
  domset1D& operator-=(domset1D const& other)
  {
    if (!m_set.unique())
      m_set.reset(new set_type(*m_set));
    *m_set -= *other.m_set;
    m_cont_dom = false;
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the given @p index from the domain.
  //////////////////////////////////////////////////////////////////////
  domset1D& operator-=(index_type const& index)
  {
    if (!m_set.unique())
      m_set.reset(new set_type(*m_set));
    *m_set -= index;
    m_cont_dom = false;
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the union between this domain and the given @p
  ///        other domain.
  //////////////////////////////////////////////////////////////////////
  domset1D operator+(domset1D const& other) const
  {
    return { *m_set + *other.m_set };
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the intersection between this domain and the
  ///        given @p other domain.
  //////////////////////////////////////////////////////////////////////
  domset1D operator&(domset1D const& other) const
  {
    return { *m_set & *other.m_set };
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the intersection between this domain and the
  ///        given @p other consecutive index domain (e.g.,
  ///        indexed_domain).
  /// @todo Add constraints on the type accepted.
  //////////////////////////////////////////////////////////////////////
  template <typename ODom>
  domset1D operator&(ODom const& other) const
  {
    typename ODom::index_type olower = other.first();
    typename ODom::index_type oupper = other.last();

    if (oupper < olower)
      std::swap(olower, oupper);

    set_type oset(int_type::closed(olower, oupper));

    return { *m_set & oset };
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::advance(index_type, D)
  //////////////////////////////////////////////////////////////////////
  template <typename Distance>
  index_type advance(index_type index, Distance const& n) const
  {
    if (is_contiguous())
      return (index + n) > boost::icl::last(*m_set->begin()) ?
        index_bounds<index_type>::invalid() : (index + n);

    typename set_type::iterator it = m_set->find(index);
    typename set_type::iterator nit = it;

    // Make sure we do not change the constant distance argument passed to this
    // method (Distance might be a proxy backed by a read-only container without
    // the operator-=).
    index_type dist = n;

    while (nit != m_set->end()) {
      index_type upper_e = boost::icl::last(*nit);
      index_type new_idx = index + dist;
      if (new_idx <= upper_e)
        return new_idx;
      else {
        ++nit;
        if (nit != m_set->end()) {
          dist -= (upper_e - index + 1);
          index = boost::icl::first(*nit);
        }
      }
    }

    return index_bounds<index_type>::invalid();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::distance(index_type const&, index_type const&)
  //////////////////////////////////////////////////////////////////////
  size_type distance(index_type const& i0, index_type const& i1) const
  {
    if (i0<=i1)
      return boost::icl::size(*m_set & int_type(i0,i1));
    else
      return boost::icl::size(*m_set & int_type(i1,i0));
  }

  bool is_contiguous(void) const
  {
    return boost::icl::interval_count(*m_set) == 1;
  }

#ifdef _STAPL
  void define_type(typer& t)
  {
    t.member(m_set);
    t.member(m_cont_dom);
  }
#endif

  template<typename IT>
  friend std::ostream& operator<<(std::ostream&, domset1D<IT> const&);
}; // class domset1D


template<typename T>
std::ostream& operator<<(std::ostream& os, domset1D<T> const& d)
{
  if (d.empty())
    return os << "empty";
  return os << *d.m_set;
}


template <typename T>
struct is_domain_sparse;


template <typename T>
struct is_domain_sparse<domset1D<T> >
  : boost::mpl::true_
{ };

} // namespace stapl

#endif // STAPL_DOMAINS_INTERVAL_HPP
