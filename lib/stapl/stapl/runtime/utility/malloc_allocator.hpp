/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_MALLOC_ALLOCATOR_HPP
#define STAPL_RUNTIME_UTILITY_MALLOC_ALLOCATOR_HPP

#include <cstdlib>
#include <memory>
#include <new>
#include <utility>

namespace stapl {

namespace runtime {

template<typename T>
class malloc_allocator;


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref malloc_allocator for @c void.
///
/// See C++11 Draft n3337 20.6.9
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<>
class malloc_allocator<void>
{
public:
  typedef void*       pointer;
  typedef const void* const_pointer;
  typedef void        value_type;

  template<typename U>
  struct rebind
  {
    typedef malloc_allocator<U> other;
  };
};


//////////////////////////////////////////////////////////////////////
/// @brief An allocator that uses @c std::malloc() directly.
///
/// See The C++ Programming Language, 3rd edition 19.4 and C++11 Draft 20.9.5
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename T>
class malloc_allocator
{
public:
  typedef std::size_t    size_type;
  typedef std::ptrdiff_t difference_type;
  typedef T*             pointer;
  typedef const T*       const_pointer;
  typedef T&             reference;
  typedef T const&       const_reference;
  typedef T              value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Provides a way to obtain an allocator for a different type.
  //////////////////////////////////////////////////////////////////////
  template<typename U>
  struct rebind
  {
    typedef malloc_allocator<U> other;
  };

  malloc_allocator(void) noexcept = default;
  malloc_allocator(malloc_allocator const&) noexcept = default;

  template<typename U>
  malloc_allocator(malloc_allocator<U> const&) noexcept
  { }

  pointer address(reference x) const noexcept
  { return std::addressof(x); }

  const_pointer address(const_reference x) const noexcept
  { return std::addressof(x); }

  pointer allocate(size_type n,
                   malloc_allocator<void>::const_pointer /*hint*/ = 0)
  {
    pointer p = static_cast<pointer>(std::malloc(n * sizeof(value_type)));
    if (!p)
      throw std::bad_alloc();
    return p;
  }

  void deallocate(pointer p, size_type /*n*/)
  { std::free(p); }

  size_type max_size(void) const noexcept
  { return (size_type(-1)/sizeof(value_type)); }

  void construct(pointer p, const_reference val)
  { ::new(static_cast<void*>(p)) value_type(val); }

  void destroy(pointer p)
  { p->~value_type(); }

  template<typename U, typename... Args>
  void construct(U* p, Args&&... args)
  { ::new(static_cast<void*>(p)) value_type(std::forward<Args>(args)...); }

  template<typename U>
  void destroy(U* p)
  { p->~U(); }
};


template<typename T1, typename T2>
constexpr bool operator==(malloc_allocator<T1> const&,
                          malloc_allocator<T2> const&) noexcept
{
  return true;
}

template <typename T1, typename T2>
constexpr bool operator!=(malloc_allocator<T1> const&,
                          malloc_allocator<T2> const&) noexcept
{
  return false;
}

} // namespace runtime

} // namespace stapl

#endif
