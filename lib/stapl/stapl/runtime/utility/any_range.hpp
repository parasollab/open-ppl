/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_ANY_RANGE_HPP
#define STAPL_RUNTIME_UTILITY_ANY_RANGE_HPP

#include "../exception.hpp"
#include <iterator>
#include <utility>
#include <boost/range/any_range.hpp>
#include <boost/range/size.hpp>

namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Type erased range of const objects with associated size information.
///
/// @tparam T Range object type.
///
/// @ingroup runtimeUtility
////////////////////////////////////////////////////////////////////
template<typename T>
class any_range
{
public:
  using size_type      = std::size_t;
private:
  using range_type     = boost::any_range<T,
                                          boost::forward_traversal_tag,
                                          const T,
                                          std::ptrdiff_t>;
public:
  using const_iterator = typename range_type::const_iterator;
  using iterator       = const_iterator;

private:
  size_type  m_size;
  range_type m_range;

public:
  any_range(void)
  : m_size(0)
  { }

  template<typename U>
  any_range(U&& u)
  : m_size(boost::size(u)),
    m_range(std::forward<U>(u))
  { }

  template<typename U>
  any_range(U&& u, const size_type size)
  : m_size(size),
    m_range(std::forward<U>(u))
  { STAPL_RUNTIME_ASSERT(boost::size(m_range)==m_size); }

  bool empty(void) const noexcept
  { return (m_size==0); }

  size_type size(void) const
  { return m_size; }

  const_iterator begin(void) const noexcept
  { return m_range.begin(); }

  const_iterator end(void) const noexcept
  { return m_range.end(); }
};

} // namespace runtime

} // namespace stapl

#endif
