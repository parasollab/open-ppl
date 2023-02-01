/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_ALGORITHM_HPP
#define STAPL_RUNTIME_UTILITY_ALGORITHM_HPP

#include "../exception.hpp"
#include <climits>
#include <iterator>
#include <unordered_set>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Finds index of the given element in a range.
///
/// @tparam Range Range type.
/// @tparam T     Element type.
///
/// @warning This function has \f$O(n)\f$ time complexity.
///
/// @param r Range to find the element in.
/// @param v Element to be found.
///
/// @return The index of @p v in @p r. If not found, returns @c r.size().
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename Range, typename T>
auto find_index(Range const& r, T const& v) noexcept
  -> typename std::iterator_traits<decltype(std::begin(r))>::difference_type
{
  typename std::iterator_traits<decltype(std::begin(r))>::difference_type i = 0;
  for (auto&& t : r) {
    if (t==v)
      return i;
    ++i;
  }
  return ++i;
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns @c true if the range has unique elements, otherwise returns
///        @c false.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename Range>
bool all_unique(Range const& r)
{
  typedef typename std::iterator_traits<
            decltype(std::begin(r))
          >::value_type value_type;

  std::unordered_set<value_type> s;
  for (auto&& t : r) {
    if (!s.insert(t).second)
      return false;
  }
  return true;
}


//////////////////////////////////////////////////////////////////////
/// @brief Log2 algorithm for unsigned integral types.
///
/// From http://graphics.stanford.edu/~seander/bithacks.html
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename UIntType>
UIntType integral_ceil_log2(UIntType i) noexcept
{
  const UIntType t = ((i & (i-1)) ? 1 : 0); // find if power-of-2
  UIntType r = 0;
  while (i>>=1)
    ++r;
  return (r + t);
}


//////////////////////////////////////////////////////////////////////
/// @brief Sets the [@p msb, @p lsb] bits of @p dst to value @p v and returns
///        the new @p dst.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename UIntType, typename Size>
UIntType set_bits(UIntType dst, UIntType v, Size msb, Size lsb) noexcept
{
  STAPL_RUNTIME_ASSERT(msb>=lsb && (sizeof(UIntType) * CHAR_BIT)>msb);
  const Size len      = (msb - lsb + 1); // inclusive range
  const UIntType mask = ((UIntType(0x1) << len) - UIntType(0x1)) << lsb;
  return ((dst & ~mask) | ((v << lsb) & mask));
}


//////////////////////////////////////////////////////////////////////
/// @brief Reads the [@p msb, @p lsb] bits of @p dst.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename UIntType, typename Size>
UIntType read_bits(UIntType dst, Size msb, Size lsb) noexcept
{
  STAPL_RUNTIME_ASSERT(msb>=lsb && (sizeof(UIntType) * CHAR_BIT)>msb);
  const Size len      = (msb - lsb + 1); // inclusive range
  const UIntType mask = ((UIntType(0x1) << len) - UIntType(0x1));
  return ((dst >> lsb) & mask);
}

} // namespace runtime

} // namespace stapl

#endif
