/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_ARRAY_HPP
#define STAPL_RUNTIME_SERIALIZATION_ARRAY_HPP

#include "typer_fwd.hpp"
#include "../type_traits/is_basic.hpp"
#include <cstring>
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits for C arrays of non-basic @p T.
///
/// @tparam T    Array element type.
/// @tparam Size Size of the array.
///
/// @see is_basic
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T, std::size_t Size>
class typer_traits<T[Size],
                   typename std::enable_if<!is_basic<T>::value>::type>
{
public:
  using value_type = T[Size];

  static std::size_t packed_size(value_type const& t) noexcept
  {
    typer ct{typer::SIZE};
    for (std::size_t i=0; i<Size; ++i)
      ct.member(t[i]);
    return ct.offset();
  }

  static std::pair<bool, std::size_t>
  meets_requirements(const typer::pass_type p, value_type const& t) noexcept
  {
    typer ct{p};
    for (std::size_t i=0; i<Size; ++i)
      ct.member(t[i]);
    return ct.meets_requirements();
  }

  static void prepack(value_type* dest,
                      value_type const* src,
                      const std::size_t num = 1) noexcept
  {
    std::memcpy(static_cast<void*>(dest),
                static_cast<void const*>(src),
                (sizeof(value_type) * num));
  }

  static std::size_t pack(value_type& dest,
                          void* base,
                          const std::size_t offset,
                          value_type const& src) noexcept
  {
    typer ct{dest, src, base, offset};
    for (std::size_t i=0; i<Size; ++i)
      ct.member(dest[i]);
    return (ct.offset() - offset); // actual packed size is needed, not total
  }

  static std::size_t unpack(value_type& t, void* base)
  {
    typer ct{base};
    for (std::size_t i=0; i<Size; ++i)
      ct.member(t[i]);
    return ct.offset();
  }

  static void destroy(value_type& t) noexcept
  {
    typer ct{typer::DESTROY};
    for (std::size_t i=0; i<Size; ++i)
      ct.member(t[i]);
  }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref typer_traits_specialization for C arrays.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename T, std::size_t Size>
struct typer_traits_specialization<T[Size],
                                   typename std::enable_if<
                                     !is_basic<T>::value
                                   >::type>
: public std::true_type
{ };

} // namespace stapl

#endif
