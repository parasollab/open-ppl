/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_NEW_HPP
#define STAPL_RUNTIME_NEW_HPP

#include "config/find_user_config.hpp"
#include "exception.hpp"
#include "utility/option.hpp"
#include <new>
#ifdef STAPL_ENABLE_HEAP_TRACKING
# include "heap_tracker.hpp"
#endif

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Provides interfaces to do pool allocation for objects.
///
/// The allocation and deallocation are not thread-safe, but they are
/// thread-local.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
class memory_allocator
{
public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes the @ref memory_allocator.
  ///
  /// @param opts @ref option object to initialize the memory allocator with.
  //////////////////////////////////////////////////////////////////////
  static void initialize(option const& opts);

  //////////////////////////////////////////////////////////////////////
  /// @brief Finalizes the @ref memory_allocator.
  ///
  /// It will print allocation information if
  /// @c STAPL_ENABLE_MANAGED_ALLOC_STATISTICS is set.
  //////////////////////////////////////////////////////////////////////
  static void finalize(void);

  //////////////////////////////////////////////////////////////////////
  /// @brief Allocates the requested number of bytes.
  ///
  /// It will keep track of the allocations if
  /// @c STAPL_ENABLE_MANAGED_ALLOC_STATISTICS is set.
  ///
  /// @todo It needs alignment information.
  //////////////////////////////////////////////////////////////////////
  static void* allocate(std::size_t) throw(std::bad_alloc);

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the allocated storage.
  ///
  /// It will keep track of the allocations if
  /// @c STAPL_ENABLE_MANAGED_ALLOC_STATISTICS is set.
  //////////////////////////////////////////////////////////////////////
  static void deallocate(void*, std::size_t);
};


namespace new_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Normalizes small number of bytes to specific buckets.
///
/// If the number of bytes is less that 16, 32, 64 or 128, then it goes to the
/// 16-byte bucket, 32-byte bucket, 64-byte bucket or 128-byte bucket
/// respectively.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
constexpr std::size_t normalize_size(const std::size_t s) noexcept
{
  return (s<=16 ? 16 : (s<=32 ? 32 : (s<=64 ? 64 : s<=128 ? 128 : s)));
}

} // namespace new_impl

} // namespace runtime

} // namespace stapl


//////////////////////////////////////////////////////////////////////
/// @brief Enables allocation through a pool back by the
///        @ref stapl::runtime::memory_allocator for the class that this macro
///        is in.
///
/// @warning This macro has to be after a public storage specifier.
///
/// @ingroup runtimeUtility
///
/// @todo It does not support array allocation.
/// @todo It requires some additions to track the type of the object, not just
///       the size.
//////////////////////////////////////////////////////////////////////
#ifndef STAPL_RUNTIME_MANAGED_ALLOC_DISABLE
#define STAPL_USE_MANAGED_ALLOC(class_name)                                   \
  static void* operator new(std::size_t size) throw(std::bad_alloc)           \
  {                                                                           \
    STAPL_RUNTIME_ASSERT_MSG(size==sizeof(class_name), "sizes do not match"); \
    const std::size_t ns = stapl::runtime::new_impl::normalize_size(size);    \
    return stapl::runtime::memory_allocator::allocate(ns);                    \
  }                                                                           \
                                                                              \
  static void operator delete(void* p, std::size_t size) throw()              \
  {                                                                           \
    STAPL_RUNTIME_ASSERT_MSG(size==sizeof(class_name), "sizes do not match"); \
    const std::size_t ns = stapl::runtime::new_impl::normalize_size(size);    \
    return stapl::runtime::memory_allocator::deallocate(p, ns);               \
  }
#else
#define STAPL_USE_MANAGED_ALLOC(class_name) 
#endif

#endif
