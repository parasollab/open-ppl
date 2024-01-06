/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_TREE_HPP
#define STAPL_RUNTIME_UTILITY_TREE_HPP

#include "algorithm.hpp"
#include <cmath>
#include <tuple>
#include <utility>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/range/irange.hpp>
#include <boost/range/iterator_range.hpp>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Returns the root, parent and children ids for the part of a flat tree
///        that @p myid belongs to.
///
/// @tparam IntType Integral type.
///
/// @param myid Id for which the part of the tree will be generated.
/// @param n    Total number of ids.
///
/// @return A tuple with the root, parent and children ids as a range.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename IntType, typename Size>
std::tuple<IntType, IntType, decltype(boost::irange(std::declval<IntType>(),
                                                    std::declval<IntType>()))>
make_flat_tree(IntType myid, Size n)
{
  return (myid !=0 ? std::make_tuple(IntType(0),
                                     IntType(0),
                                     boost::irange(IntType(0), IntType(0)))
                   : std::make_tuple(IntType(0),
                                     IntType(0),
                                     boost::irange(IntType(1), IntType(n))));
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns the root, parent and children ids for the part of a binary
///        tree that @p myid belongs to.
///
/// @tparam IntType Integral type.
///
/// @param myid Id for which the part of the tree will be generated.
/// @param n    Total number of ids.
///
/// @return A tuple with the root, parent and children ids.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename IntType, typename Size>
std::tuple<IntType, IntType, decltype(boost::irange(std::declval<IntType>(),
                                                    std::declval<IntType>()))>
make_binary_tree(IntType myid, Size n)
{
  const IntType parent = ((myid==0) ? 0 : ((myid-1)/2));
  const Size left      = (2*(myid+1) - 1);
  const Size right     = (2*(myid+1));
  if (left<n) {
    if (right<n) {
      return std::make_tuple(IntType(0),
                             parent,
                             boost::irange(IntType(left), IntType(right + 1)));
    }
    else {
      return std::make_tuple(IntType(0),
                             parent,
                             boost::irange(IntType(left), IntType(left + 1)));
    }
  }
  return std::make_tuple(IntType(0),
                         parent,
                         boost::irange(IntType(0), IntType(0)));

}


//////////////////////////////////////////////////////////////////////
/// @brief Iterator to generate range of children ids required from
///        @ref make_binomial_tree.
///
/// @tparam IntType Integral type.
/// @tparam Size    Size type.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename IntType, typename Size>
class binomial_tree_iterator
: public boost::iterator_facade<
           binomial_tree_iterator<IntType, Size>,
           IntType,
           boost::forward_traversal_tag,
           IntType>
{
private:
  friend class boost::iterator_core_access;

  IntType m_id;
  Size    m_idx;

public:
  constexpr binomial_tree_iterator(void) noexcept
  : m_id(),
    m_idx(0)
  { }

  constexpr binomial_tree_iterator(const IntType id, const Size index) noexcept
  : m_id(id),
    m_idx(index)
  { }

private:
  IntType dereference(void) const
  { return (m_id + m_idx); }

  bool equal(binomial_tree_iterator const& other) const noexcept
  { return (m_idx==other.m_idx); }

  void increment(void) noexcept
  { m_idx /= 2; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns the root, parent and children ids for the part of a binomial
///        tree that @p myid belongs to.
///
/// @tparam IntType Integral type.
///
/// @param myid Id for which the part of the tree will be generated.
/// @param n    Total number of ids.
///
/// @return A tuple with the root, parent and children ids.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename IntType, typename Size>
std::tuple<IntType,
           IntType,
           boost::iterator_range<binomial_tree_iterator<IntType, Size>>>
make_binomial_tree(IntType myid, Size n)
{
  using iterator_type = binomial_tree_iterator<IntType, Size>;

  // round up to the next highest power of 2
  const Size N2 = (0x1 << integral_ceil_log2(n));

  // find level in the tree and parent
  IntType parent    = 0;
  Size level        = 1;
  IntType tmpParent = 0;
  for (Size tmp = N2/(0x1<<level); tmpParent!=myid; ++level, tmp /= 2) {
    if (myid>=(tmpParent+tmp)) {
      parent     = tmpParent;
      tmpParent += tmp;
    }
  }

  // find the first valid child id
  for (Size first = N2/(0x1<<level); first >= 1; first /= 2) {
    const auto t = (myid + first);
    if (t < n)
      return std::make_tuple(IntType(0),
                             parent,
                             boost::make_iterator_range(
                               iterator_type{myid, first}, iterator_type{}));
  }

  // has no children
  return std::make_tuple(IntType(0),
                         parent,
                         boost::make_iterator_range(
                           iterator_type{}, iterator_type{}));
}

} // namespace runtime

} // namespace stapl

#endif
