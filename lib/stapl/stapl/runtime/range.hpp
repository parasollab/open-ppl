/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RANGE_HPP
#define STAPL_RUNTIME_RANGE_HPP

#include "serialization.hpp"
#include "type_traits/is_movable.hpp"
#include "type_traits/is_contiguous_iterator.hpp"
#include <iterator>
#include <memory>
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Describes a range of @c const objects.
///
/// @tparam Iterator Container iterator type.
///
/// This range is used to that only a part of a container needs to be
/// communicated.
///
/// @see immutable_range_wrapper
/// @ingroup ARMIUtilities
///
/// @todo Ranges over non-contiguous containers are not yet supported.
//////////////////////////////////////////////////////////////////////
template<typename Iterator, typename = void>
class range_wrapper;


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref range_wrapper for contiguous iterators.
///
/// @see is_contiguous_iterator
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename Iterator>
class range_wrapper<Iterator,
                    typename std::enable_if<
                      runtime::is_contiguous_iterator<Iterator>::value
                    >::type>
{
public:
  typedef typename std::iterator_traits<Iterator>::value_type value_type;
  typedef std::size_t                                         size_type;
  typedef value_type const*                                   const_iterator;

private:
  const size_type   m_size;
  value_type const* m_p;

public:
  range_wrapper(Iterator first, Iterator last) noexcept
  : m_size(std::distance(first, last)),
    m_p(std::addressof(*first))
  { }

  range_wrapper(Iterator first, const std::size_t size) noexcept
  : m_size(size),
    m_p(std::addressof(*first))
  { }

  const_iterator begin(void) const noexcept
  { return const_iterator{m_p}; }

  const_iterator end(void) const noexcept
  { return const_iterator{m_p + m_size}; }

  const_iterator cbegin(void) const noexcept
  { return begin(); }

  const_iterator cend(void) const noexcept
  { return end(); }

  bool empty(void) const noexcept
  { return (m_size==0); }

  size_type size(void) const noexcept
  { return m_size; }

  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_p, m_size);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Creates a wrapper over the range <tt>[first, last)</tt>.
///
/// Upon communication, a copy of the elements in the range is given to the
/// callee.
///
/// @see make_immutable_range
/// @related range_wrapper
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename InputIterator>
range_wrapper<InputIterator>
make_range(InputIterator first, InputIterator last) noexcept
{
  return range_wrapper<InputIterator>{first, last};
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates a wrapper over the range <tt>[first, first + n)</tt>.
///
/// Upon communication, a copy of the elements in the range is given to the
/// callee.
///
/// @see make_immutable_range_n
/// @related range_wrapper
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename InputIterator, typename Size>
range_wrapper<InputIterator>
make_range_n(InputIterator first, const Size count) noexcept
{
  return range_wrapper<InputIterator>(first, count);
}


namespace runtime {

////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_movable for @c range_wrapper.
///
/// A @ref range_wrapper objects is considered non-movable so that in shared
/// memory, the range of objects is always copied instead of passed by
/// reference.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename Iterator>
struct is_movable<range_wrapper<Iterator>>
: public std::false_type
{ };

} // namespace runtime

} // namespace stapl

#endif
