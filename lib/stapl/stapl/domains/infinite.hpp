/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_INFINITE_DOMAIN_HPP
#define STAPL_DOMAINS_INFINITE_DOMAIN_HPP

#include <cstddef>
#include <iostream>

#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/utility/vs_map.hpp>

namespace stapl {

template<typename T>
struct minus;

namespace infinite_impl {

template <typename T>
struct plus
{
private:
  T m_operand;

public:
  typedef T result_type;

  plus(T operand = index_bounds<T>::invalid())
    : m_operand(operand)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Addition of a scalar constant to every element in
  /// a tuple.
  ///
  /// Used to form the GID of the last element of a domain given the size
  /// of the domain.
  //////////////////////////////////////////////////////////////////////
  template <typename Ref>
  T operator()(Ref const& ref) const
  { return ref + m_operand; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Element-wise addition on a pair of tuples.
  //////////////////////////////////////////////////////////////////////
  template <typename Ref>
  T operator()(Ref const& lhs, Ref const& rhs) const
  { return lhs + rhs; }
}; // struct plus


//////////////////////////////////////////////////////////////////////
/// @brief Common base class of infinite domains used in implementation
///  of has_finite_domain metafunction.
//////////////////////////////////////////////////////////////////////
struct infinite_base
{ };

} // namespace infinite_impl


//////////////////////////////////////////////////////////////////////
/// @brief Represents a one dimensional infinite domain.
//////////////////////////////////////////////////////////////////////
class infinite
  : public infinite_impl::infinite_base
{
public:
  typedef std::size_t index_type;
  typedef index_type  gid_type;
  typedef std::size_t size_type;

  infinite(void) = default;

  infinite(index_type const&, index_type const&)
  { }

  infinite(index_type const&, index_type const&, infinite)
  { }

  //////////////////////////////////////////////////////////////////////
  // common functionality for all domains
  //////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::first
  //////////////////////////////////////////////////////////////////////
  index_type first(void) const
  {
    return index_bounds<index_type>::lowest();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::last
  //////////////////////////////////////////////////////////////////////
  index_type last(void) const
  {
    return index_bounds<index_type>::highest();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::open_last
  //////////////////////////////////////////////////////////////////////
  index_type open_last(void) const
  {
    return index_bounds<index_type>::highest();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::contains
  //////////////////////////////////////////////////////////////////////
  bool contains(index_type const&) const
  {
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::size
  //////////////////////////////////////////////////////////////////////
  size_type size(void) const
  {
    return index_bounds<index_type>::highest();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::empty
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::is_same_container_domain
  //////////////////////////////////////////////////////////////////////
  bool is_same_container_domain(void) const
  {
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "INFINITE_DOMAIN %p: " << this;
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

  template<typename Distance>
  index_type advance(index_type idx, Distance n) const
  {
    if (idx==last())
      return idx;
    return (idx+n);
  }

  size_type distance(index_type const& i0, index_type const& i1) const
  {
    if (i0>=i1)
      return (i0-i1);
    return (i1-i0);
  }

  infinite operator&(infinite const&)
  {
    return *this;
  }
}; // class infinite


//////////////////////////////////////////////////////////////////////
/// @brief Represents an n-dimensional infinite domain.
//////////////////////////////////////////////////////////////////////
template <int N>
class infinite_nd
  : public infinite_impl::infinite_base
{
public:
  typedef std::integral_constant<int, N>                         dimension_type;
  typedef typename homogeneous_tuple_type<N, std::size_t>::type  index_type;
  typedef typename homogeneous_tuple_type<N, std::size_t>::type  gid_type;
  typedef typename homogeneous_tuple_type<N, std::size_t>::type  size_type;
  typedef typename default_traversal<N>::type                    traversal_type;
  typedef std::size_t linear_size_type;

  infinite_nd(void) = default;

  infinite_nd(index_type const&, index_type const&)
  { }

  infinite_nd(index_type const&, index_type const&, infinite)
  { }

  template<typename Ignored>
  infinite_nd(Ignored&&)
  { }

  //////////////////////////////////////////////////////////////////////
  // common functionality for all domains
  //////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::first
  //////////////////////////////////////////////////////////////////////
  index_type first(void) const
  {
    return index_bounds<index_type>::lowest();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::last
  //////////////////////////////////////////////////////////////////////
  index_type last(void) const
  {
    return index_bounds<index_type>::highest();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::open_last
  //////////////////////////////////////////////////////////////////////
  index_type open_last(void) const
  {
    return index_bounds<index_type>::highest();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::contains
  //////////////////////////////////////////////////////////////////////
  bool contains(index_type const&) const
  {
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::contains
  //////////////////////////////////////////////////////////////////////
  template <typename... Indices>
  bool contains(Indices...) const
  {
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::size
  //////////////////////////////////////////////////////////////////////
  linear_size_type size(void) const
  {
    return index_bounds<std::size_t>::highest();
  }

  size_type dimensions(void) const
  {
    return index_bounds<size_type>::highest();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::empty
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::is_same_container_domain
  //////////////////////////////////////////////////////////////////////
  bool is_same_container_domain(void) const
  {
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "INFINITE_DOMAIN_ND %p: " << this;
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

  template<typename Distance>
  index_type advance(index_type idx, Distance n) const
  {
    if (idx == last())
      return idx;

    return vs_map(stapl::infinite_impl::plus<Distance>(n), idx);
  }

  size_type distance(index_type const& i0, index_type const& i1) const
  {
    if (i0 >= i1)
      return vs_map(stapl::minus<index_type>(), i0, i1);

    return vs_map(stapl::minus<index_type>(), i1, i0);
  }

  infinite_nd operator&(infinite_nd const&) const
  {
    return *this;
  }
}; // class infinite


inline std::ostream& operator<<(std::ostream &os, infinite const&)
{
  return os << "[Infinite]";
}


template<int N>
std::ostream& operator<<(std::ostream &os, infinite_nd<N> const&)
{
  return os << "[Infinite_ND<" << N << ">]";
}

} // namespace stapl

#endif // STAPL_DOMAINS_INFINITE_DOMAIN_HPP
