/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_STD_ARRAY_HPP
#define STAPL_RUNTIME_SERIALIZATION_STD_ARRAY_HPP

#include "typer_fwd.hpp"
#include <type_traits>

namespace std {

template<typename T, std::size_t Size>
struct array;

} // namespace std

namespace boost {

template<typename T, std::size_t Size>
class array;

} // namespace boost


namespace stapl {

////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref define_type_provider for @c std::array.
///
/// @ingroup serialization
////////////////////////////////////////////////////////////////////
template<typename T, std::size_t Size>
struct define_type_provider<std::array<T, Size>>
{
  //////////////////////////////////////////////////////////////////////
  /// @brief @c std::array doppelganger that provides @c define_type().
  //////////////////////////////////////////////////////////////////////
  struct wrapper
  : public std::array<T, Size>
  {
    void define_type(typer& t)
    {
      for (std::size_t i = 0; i < Size; ++i)
        t.member(this->operator[](i));
    }
  };

  static constexpr wrapper& apply(std::array<T, Size>& t) noexcept
  {
    return static_cast<wrapper&>(t);
  }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref define_type_provider for @c boost::array.
///
/// @ingroup serialization
////////////////////////////////////////////////////////////////////
template<typename T, std::size_t Size>
struct define_type_provider<boost::array<T, Size>>
{
  //////////////////////////////////////////////////////////////////////
  /// @brief @c boost::array doppelganger that provides @c define_type().
  //////////////////////////////////////////////////////////////////////
  struct wrapper
  : public boost::array<T, Size>
  {
    void define_type(typer& t)
    {
      for (std::size_t i = 0; i < Size; ++i)
        t.member(this->operator[](i));
    }
  };

  static constexpr wrapper& apply(boost::array<T, Size>& t) noexcept
  {
    return static_cast<wrapper&>(t);
  }
};

} // namespace stapl

#endif
