/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_PAIR_HPP
#define STAPL_RUNTIME_SERIALIZATION_PAIR_HPP

#include "typer_fwd.hpp"
#include <type_traits>

namespace std {

template<typename T1, typename T2>
struct pair;

} // namespace std


namespace stapl {

////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref define_type_provider for @c std::pair.
///
/// @ingroup serialization
////////////////////////////////////////////////////////////////////
template<typename T1, typename T2>
struct define_type_provider<std::pair<T1, T2>>
{
  static_assert(!std::is_reference<T1>::value && !std::is_reference<T1>::value,
                "Reference packing is not allowed.");

  //////////////////////////////////////////////////////////////////////
  /// @brief @c std::pair doppelganger that provides @c define_type().
  //////////////////////////////////////////////////////////////////////
  struct wrapper
  : public std::pair<T1, T2>
  {
    void define_type(typer& t)
    {
      t.member(this->first);
      t.member(this->second);
    }
  };

  static constexpr wrapper& apply(std::pair<T1, T2>& t) noexcept
  {
    return static_cast<wrapper&>(t);
  }
};

} // namespace stapl

#endif
