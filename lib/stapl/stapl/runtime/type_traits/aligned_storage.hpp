/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_ALIGNED_STORAGE_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_ALIGNED_STORAGE_HPP

#include "../config/platform.hpp"
#include <type_traits>

namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Provides the member typedef type, which is a POD type suitable for
///        use as uninitialized storage for any object whose size is at most
///        @p Len and whose alignment requirement is a divisor of
///        @ref STAPL_RUNTIME_DEFAULT_ALIGNMENT.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<std::size_t Len>
using aligned_storage_t =
  typename std::aligned_storage<Len, STAPL_RUNTIME_DEFAULT_ALIGNMENT>::type;


////////////////////////////////////////////////////////////////////
/// @brief Returns @p n adjusted with extra padding bytes to satisfy alignment
///        of @p alignment.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
constexpr std::size_t
aligned_size(const std::size_t n,
             const std::size_t alignment =
               STAPL_RUNTIME_DEFAULT_ALIGNMENT) noexcept
{
  return (n + /* padding */ (alignment - ((n - 1) % alignment + 1)));
}

} // namespace runtime

} // namespace stapl

#endif
