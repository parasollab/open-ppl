/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CONST_LOCAL_ITERATOR_HPP
#define STAPL_CONTAINERS_CONST_LOCAL_ITERATOR_HPP

#include <stapl/containers/iterators/const_local_accessor.hpp>
#include <stapl/views/iterator/iterator_facade.h>
#include <stapl/utility/use_default.hpp>

#include <valarray>

namespace stapl {

template <typename C>
struct get_base_const_iterator
{
  typedef typename C::const_iterator type;
};


template <typename T>
struct get_base_const_iterator<std::valarray<T> >
{
  typedef T const* type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Iterator that is used to locally traverse a base container.
///
/// @tparam Container Type of the base container
/// @tparam Accessor Accessor type used for references (proxies). By default,
/// @ref local_accessor will be chosen.
///
/// @see proxy
//////////////////////////////////////////////////////////////////////
template<typename C, typename A = use_default>
class const_local_iterator
  : public iterator_facade<
      const_local_iterator<C, A>,
      typename select_parameter<A, const_local_accessor<C> >::type
    >
{
private:
  friend class iterator_core_access;

  typedef typename C::container_type                        cont_t;
  typedef typename get_base_const_iterator<cont_t>::type    base_iter_t;
  typedef typename C::value_type                            value_t;
  typedef typename std::iterator_traits<base_iter_t>        traits_t;
  typedef typename traits_t::difference_type                diff_t;

  typedef typename select_parameter<
    A, local_accessor<C> >::type                            accessor_t;

  base_iter_t       m_iter;
  C const*          m_cont;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create an invalid iterator.
  //////////////////////////////////////////////////////////////////////
  const_local_iterator(void)
    : m_cont(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a local iterator over a given container starting at
  /// a given base container iterator.
  /// @param iter Iterator from the base container to the element
  /// @param cont Pointer to the container
  //////////////////////////////////////////////////////////////////////
  const_local_iterator(base_iter_t const& iter, C const* cont)
    : m_iter(iter), m_cont(cont)
  { }

  /////////////////////////////////////////////////////////////////////
  /// @brief Standard operator->() for iterators that returns a pointer
  /// to the iterated value.
  /////////////////////////////////////////////////////////////////////
  base_iter_t operator->(void) const
  {
    return m_iter;
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create an accessor to be used in a reference for the value that
  /// the iterator is currently pointing to.
  //////////////////////////////////////////////////////////////////////
  accessor_t access(void) const
  {
    return accessor_t(m_cont,m_iter);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compare equality of two iterators by testing that they point to
  /// the same element.
  /// @param rhs The other iterator to compare against
  /// @return Whether or not these two iterators point to the same value
  //////////////////////////////////////////////////////////////////////
  bool equal(const_local_iterator const& rhs) const
  {
    return m_iter == rhs.m_iter && m_cont == rhs.m_cont;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Increment the iterator by one position.
  //////////////////////////////////////////////////////////////////////
  void increment(void)
  {
    ++m_iter;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deccrement the iterator by one position.
  //////////////////////////////////////////////////////////////////////
  void decrement(void)
  {
    --m_iter;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Advance the iterator forward or backward by a specific
  /// amount.
  /// @param n The amount by which to advance the iterator
  //////////////////////////////////////////////////////////////////////
  void advance(diff_t n)
  {
    std::advance(m_iter, n);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the distance between two iterators by evaluating
  /// the distance of their raw iterators.
  /// @param rhs The other iterator to compare against
  /// @return How far apart the iterators are
  //////////////////////////////////////////////////////////////////////
  diff_t distance_to(const_local_iterator const& rhs) const
  {
    return std::distance(m_iter, rhs.m_iter);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determine whether this iterator precedes another in a proper
  /// traversal of the space.
  /// @param rhs The other iterator to compare against
  /// @return Whether or not this iterator precedes the other
  //////////////////////////////////////////////////////////////////////
  bool less_than(const_local_iterator const& rhs) const
  {
    return m_iter < rhs.m_iter;
  }
}; // class const_local_iterator


template<typename C, typename A>
std::ostream& operator<<(std::ostream &os,
                         const_local_iterator<C,A> const& it)
{
  os << "(" << *(it.operator->()) << ")";
  return os;
}

} // namespace stapl

#endif
