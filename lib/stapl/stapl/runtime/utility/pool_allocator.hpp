/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_POOL_ALLOCATOR_HPP
#define STAPL_RUNTIME_UTILITY_POOL_ALLOCATOR_HPP

#include "../new.hpp"
#include <memory>

namespace stapl {

template<typename T, std::size_t N = 128>
class pool_allocator;


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref pool_allocator for @c void.
///
/// See C++11 Draft n3337 20.6.9
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<std::size_t N>
class pool_allocator<void, N>
{
public:
  typedef void*       pointer;
  typedef const void* const_pointer;
  typedef void        value_type;

  template<typename U>
  struct rebind
  {
    typedef pool_allocator<U, N> other;
  };
};


//////////////////////////////////////////////////////////////////////
/// @brief Pool backed allocator.
///
/// Allocator that uses pool allocation for small (<= @p N bytes) allocations,
/// backed by the @ref runtime::memory_allocator. For larger allocations, it
/// redirects to the standard allocator.
///
/// @warning This allocator may increase your memory consumption, as memory
///          reserved by a pool cannot be used by the rest of the program.
///
/// @warning Memory allocated from this allocator can only be freed in the same
///          thread it was allocated from.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T, std::size_t N>
class pool_allocator
: private std::allocator<T>
{
private:
  template<typename, std::size_t>
  friend class pool_allocator;

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
    typedef pool_allocator<U, N> other;
  };

  pool_allocator(void) noexcept = default;
  pool_allocator(pool_allocator const&) = default;

  template<typename U>
  pool_allocator(pool_allocator<U, N> const& other) noexcept
  : std::allocator<T>(other)
  { }

  using std::allocator<T>::address;

  pointer allocate(size_type n,
                   typename pool_allocator<void, N>::const_pointer hint = 0)
  {
    using namespace runtime;

    const std::size_t sz = new_impl::normalize_size(n * sizeof(T));
    return static_cast<pointer>(sz <= N ? memory_allocator::allocate(sz)
                                        : std::allocator<T>::allocate(n, hint));
  }

  void deallocate(pointer p, size_type n)
  {
    using namespace runtime;

    const std::size_t sz = new_impl::normalize_size(n * sizeof(T));
    if (sz <= N)
      memory_allocator::deallocate(p, sz);
    else
      std::allocator<T>::deallocate(p, n);
  }

  using std::allocator<T>::max_size;
  using std::allocator<T>::construct;
  using std::allocator<T>::destroy;
};


template<typename T1, std::size_t N1, typename T2, std::size_t N2>
constexpr bool operator==(pool_allocator<T1> const&,
                          pool_allocator<T2> const&) noexcept
{
  return true;
}

template<typename T1, std::size_t N1, typename T2, std::size_t N2>
constexpr bool operator!=(pool_allocator<T1, N1> const&,
                          pool_allocator<T2, N2> const&) noexcept
{
  return false;
}

} // namespace stapl

#endif
