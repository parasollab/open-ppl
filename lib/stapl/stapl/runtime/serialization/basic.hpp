/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_BASIC_HPP
#define STAPL_RUNTIME_SERIALIZATION_BASIC_HPP

#include "typer_traits.hpp"
#include "../type_traits/is_basic.hpp"
#include <cstring>
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits for basic types.
///
/// @tparam T Object type to be packed.
///
/// @see is_basic
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
class typer_traits<T,
                   typename std::enable_if<is_basic<T>::value>::type>
{
public:
  using value_type = T;

  static constexpr std::size_t packed_size(T const&) noexcept
  { return 0; }

  static constexpr std::pair<bool, std::size_t>
  meets_requirements(const typer::pass_type, T const&) noexcept
  { return std::make_pair(true, std::size_t(0)); }

  static void prepack(T* dest,
                      T const* src,
                      const std::size_t num = 1) noexcept
  {
    if (std::is_empty<T>::value)
      return;
    std::memcpy(static_cast<void*>(dest),
                static_cast<void const*>(src),
                (sizeof(T) * num));
  }

  static constexpr std::size_t pack(T&,
                                    void*,
                                    const std::size_t,
                                    T const&) noexcept
  { return 0; }

  static constexpr std::size_t unpack(T&, void*) noexcept
  { return 0; }

  static void destroy(T&) noexcept
  { }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits_specialization for basic types.
///
/// @see is_basic
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T>
struct typer_traits_specialization<T,
                                   typename std::enable_if<
                                     is_basic<T>::value
                                   >::type>
: public std::true_type
{ };

} // namespace stapl

#endif
