/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_IMMUTABLE_RANGE_HPP
#define STAPL_RUNTIME_IMMUTABLE_RANGE_HPP

#include "serialization.hpp"
#include "type_traits/is_contiguous_iterator.hpp"
#include "type_traits/is_copyable.hpp"
#include <iterator>
#include <memory>
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Wraps a reference to a temporarily immutable range of objects.
///
/// @tparam Iterator Container iterator type.
///
/// The part of container that has been declared immutable may be passed by
/// reference when communication happens in shared memory. The container or the
/// objects in the range must not be deleted or mutated until all
/// @ref immutable_range_wrapper objects to the container have been deleted.
/// This is commonly guaranteed with synchronization.
///
/// Once all the @ref immutable_reference_wrapper objects have been destroyed,
/// then the referenced containers and its stored objects can be mutated or
/// deleted.
///
/// @see range_wrapper
/// @ingroup ARMIUtilities
///
/// @todo Ranges over non-contiguous container are not yet supported.
//////////////////////////////////////////////////////////////////////
template<typename Iterator, typename = void>
class immutable_range_wrapper;


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref immutable_range_wrapper for contiguous
///        iterators.
///
/// @see is_contiguous_iterator
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename Iterator>
class immutable_range_wrapper<Iterator,
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
  immutable_range_wrapper(Iterator first, Iterator last) noexcept
  : m_size(std::distance(first, last)),
    m_p(std::addressof(*first))
  { }

  template<typename IteratorParam>
  immutable_range_wrapper(IteratorParam first, const std::size_t size,
       typename std::enable_if<
         !std::is_pointer<IteratorParam>::value
         && std::is_same<Iterator, IteratorParam>::value
       >::type* = 0) noexcept
  : m_size(size),
    m_p(std::addressof(*first))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Allow initialization directly with pointer and size, the former
  /// which is assumed to be a @p const_iterator from another object of the
  /// same class template instance.
  //////////////////////////////////////////////////////////////////////
  immutable_range_wrapper(value_type const* p, const std::size_t size) noexcept
  : m_size(size),
    m_p(p)
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
/// @brief Creates a wrapper over the immutable range <tt>[first, last)</tt>.
///
/// An immutable range of objects may be passed by reference during
/// communication.
///
/// @warning The sender has to guarantee that all callees have finished before
///          deleting or mutating the object.
///
/// @see make_range
/// @related immutable_range_wrapper
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename InputIterator>
immutable_range_wrapper<InputIterator>
make_immutable_range(InputIterator first, InputIterator last) noexcept
{
  return immutable_range_wrapper<InputIterator>{first, last};
}


//////////////////////////////////////////////////////////////////////
/// @brief Creates a wrapper over the immutable range
///        <tt>[first, first + n)</tt>.
///
/// An immutable range of objects may be passed by reference during
/// communication.
///
/// @warning The sender has to guarantee that all callees have finished before
///          deleting or mutating the object.
///
/// @see make_range_n
/// @related immutable_range_wrapper
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename InputIterator, typename Size>
immutable_range_wrapper<InputIterator>
make_immutable_range_n(InputIterator first, const Size count) noexcept
{
  return immutable_range_wrapper<InputIterator>(first, count);
}


namespace runtime {

////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_copyable for @ref immutable_range_wrapper.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T, typename E>
struct is_copyable<immutable_range_wrapper<T, E>>
: public std::true_type
{ };

} // namespace runtime

} // namespace stapl

#endif
