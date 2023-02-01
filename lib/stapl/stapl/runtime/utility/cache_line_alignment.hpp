/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_CACHE_LINE_ALIGNMENT_HPP
#define STAPL_RUNTIME_UTILITY_CACHE_LINE_ALIGNMENT_HPP

#include <cstdlib>
#include <memory>
#include <new>
#include <utility>

namespace stapl {

namespace runtime {

template<typename T>
class cache_line_aligned_allocator;

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref cache_line_aligned_allocator for @c void.
///
/// See C++11 Draft n3337 20.6.9
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<>
class cache_line_aligned_allocator<void>
{
public:
  using pointer       = void*;
  using const_pointer = const void*;
  using value_type    = void;

  template<typename U>
  struct rebind
  {
    using other = cache_line_aligned_allocator<U>;
  };
};


//////////////////////////////////////////////////////////////////////
/// @brief An allocator that enforces that all allocated memory returned
/// to the caller begins at the start of a cache line.
///
/// Backed by malloc, this allocator overallocates enough to be able
/// to (a) advance pointer to proper alignment and (b) have storage
/// in front for original address malloc() gave to it (so that it can
/// properly call free().
///
/// @ingroup runtimeUtility
///
/// @todo Try to simplify pointer arithmetic and avoid [] without
/// sacrificing performance.  Trying to use Boost.Align (both path
/// that uses posix and the hand coded variants) are slower than this
/// version.
//////////////////////////////////////////////////////////////////////
template<typename T>
class cache_line_aligned_allocator
{
private:
  static constexpr size_t target_alignment = STAPL_RUNTIME_CACHELINE_SIZE;

public:
  using size_type       = std::size_t;
  using difference_type = std::ptrdiff_t;
  using pointer         = T*;
  using const_pointer   = const T*;
  using reference       = T&;
  using const_reference = T const&;
  using value_type      = T;

  //////////////////////////////////////////////////////////////////////
  /// @brief Provides a way to obtain an allocator for a different type.
  //////////////////////////////////////////////////////////////////////
  template<typename U>
  struct rebind
  {
    using other = cache_line_aligned_allocator<U>;
  };

  cache_line_aligned_allocator(void) noexcept = default;
  cache_line_aligned_allocator(cache_line_aligned_allocator const&) noexcept
    = default;

  template<typename U>
  cache_line_aligned_allocator(cache_line_aligned_allocator<U> const&) noexcept
  { }

  pointer address(reference x) const noexcept
  { return std::addressof(x); }

  const_pointer address(const_reference x) const noexcept
  { return std::addressof(x); }

  pointer allocate(size_type n,
                   cache_line_aligned_allocator<void>::const_pointer/*hint*/= 0)
  {
    void* malloc_addr;

    // overallocate enough to "fix" starting address and store originally
    // allocated starting address.
    const int pad = target_alignment - 1 + sizeof(void*);
    const int malloc_size = sizeof(T) * n + pad;

    if ((malloc_addr = (void*) malloc(malloc_size)) == nullptr)
      throw std::bad_alloc();

    // adjusted starting address passed to caller.
    void** adjusted_addr =
       (void**)(((size_t)(malloc_addr) + pad) & ~(target_alignment - 1));

    // store malloc() allocated starting address.
    adjusted_addr[-1] = malloc_addr;

    return static_cast<pointer>(static_cast<void*>(adjusted_addr));
  }

  void deallocate(pointer p, size_type /*n*/)
  {
    // Use stored address of actual malloc() allocation as parameter to free().
    std::free(((void**) p)[-1]);
  }

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
constexpr bool operator==(cache_line_aligned_allocator<T1> const&,
                          cache_line_aligned_allocator<T2> const&) noexcept
{
  return true;
}


template <typename T1, typename T2>
constexpr bool operator!=(cache_line_aligned_allocator<T1> const&,
                          cache_line_aligned_allocator<T2> const&) noexcept
{
  return false;
}


//////////////////////////////////////////////////////////////////////
/// @brief Storage for a single instance of type @p T.  Class has
/// @p alignas directive to ensure cache line alignment.  Used as storage
/// type for @p cache_line_aligned_vector.
///
/// @ingroup runtimeUtility
///
/// @todo Put in separate file from allocator
///////////////////////////////////////////////////////////////////////
template<typename T>
class STAPL_RUNTIME_CACHELINE_ALIGNED cache_line_aligned_storage
{
private:
  T m_value;

public:
  template<typename ...Args>
  cache_line_aligned_storage(Args&&... args)
  : m_value(std::forward<Args>(args)...)
  { }

  cache_line_aligned_storage& operator=(T const& t)
  {
    m_value = t;
    return *this;
  }

  cache_line_aligned_storage& operator=(T&& t)
  {
    m_value = std::move(t);
    return *this;
  }

  operator T const&(void) const noexcept
  { return m_value; }

  operator T&(void) noexcept
  { return m_value; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Basic vector class based on @p std::vector which enforces
/// cache line alignment.
///
/// (a) Wraps passed value_type (@p T) with @ref cache_line_aligned_storage
/// to ensure proper padding of each element.
///
/// (b) Uses @ref cache_line_aligned_allocator to ensure address of first
/// element is at start of cache line.
///
/// @ingroup runtimeUtility
///
/// @todo Put in separate file from allocator.
//////////////////////////////////////////////////////////////////////
template<typename T>
class cache_line_aligned_vector
{
private:
  using storage_t = cache_line_aligned_storage<T>;
  using vector_t  =
    std::vector<storage_t, cache_line_aligned_allocator<storage_t>>;

  vector_t m_vector;

public:
  using reference       = T&;
  using const_reference = T const&;
  using size_type       = typename vector_t::size_type;

  template<typename ...Args>
  cache_line_aligned_vector(Args&&... args)
  : m_vector(std::forward<Args>(args)...)
  { }

  reference operator[](size_type n) noexcept
  { return m_vector[n]; }

  const_reference operator[](size_type n) const noexcept
  { return m_vector[n]; }

  size_type size(void) const noexcept
  { return m_vector.size(); }
};

} // namespace runtime

} // namespace stapl

#endif
