/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_MEM_FN_HPP
#define STAPL_RUNTIME_SERIALIZATION_MEM_FN_HPP

#include "typer_fwd.hpp"
#include <type_traits>

#if defined(STAPL__GNUC__) && defined(STAPL__GNUC_MINOR__)
# if ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ <= 9) || \
      (STAPL__GNUC__ == 5) && (STAPL__GNUC_MINOR__ <= 4) || \
      (STAPL__GNUC__ == 6) && (STAPL__GNUC_MINOR__ <= 4))
# else
#  error "Only select versions of gcc 4.x and 5.x are supported"
# endif
#else
# error "Unable to determine libstdc++ version... aborting"
#endif

namespace std {

template<typename, typename...>
struct _Maybe_unary_or_binary_function;


template<typename>
class _Mem_fn;

} // namespace std


namespace stapl {

////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref define_type_provider for the result of
///        @c std::mem_fn (non-const qualified pointer to members).
///
/// @warning This implementation relies on the compiler to create identical
///          layout for two classes that are written in a similar way (identical
///          members and access control). The @c static_assert calls attempt to
///          detect any possible issues if the compiler changes its behavior.
///
/// @ingroup serialization
////////////////////////////////////////////////////////////////////
template<typename _Res, typename _Class, typename... _ArgTypes>
struct define_type_provider<std::_Mem_fn<_Res (_Class::*)(_ArgTypes...)>>
{
  //////////////////////////////////////////////////////////////////////
  /// @brief @c std::_Mem_fn doppelganger that provides @c define_type().
  //////////////////////////////////////////////////////////////////////
  class wrapper
  : public std::_Maybe_unary_or_binary_function<_Res, _Class*, _ArgTypes...>
  {
    using Functor = _Res (_Class::*)(_ArgTypes...);

    Functor __pmf;

  public:
    void define_type(typer& t)
    {
      t.base<
        std::_Maybe_unary_or_binary_function<_Res, _Class*, _ArgTypes...>
      >(*this);

      t.member(__pmf);
    }
  };

  static wrapper&
  apply(std::_Mem_fn<_Res (_Class::*)(_ArgTypes...)>& t) noexcept
  {
    static_assert(
      (sizeof(wrapper) ==
         sizeof(std::_Mem_fn<_Res (_Class::*)(_ArgTypes...)>)) &&
      (std::alignment_of<wrapper>::value ==
         std::alignment_of<std::_Mem_fn<_Res(_Class::*)(_ArgTypes...)>>::value),
      "Incompatible types.");
    return reinterpret_cast<wrapper&>(t);
  }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref define_type_provider for the result of
///        @c std::mem_fn (const qualified pointer to members).
///
/// @warning This implementation relies on the compiler to create identical
///          layout for two classes that are written in a similar way (identical
///          members and access control). The @c static_assert calls attempt to
///          detect any possible issues if the compiler changes its behavior.
///
/// @ingroup serialization
////////////////////////////////////////////////////////////////////
template<typename _Res, typename _Class, typename... _ArgTypes>
struct define_type_provider<std::_Mem_fn<_Res (_Class::*)(_ArgTypes...) const>>
{
  //////////////////////////////////////////////////////////////////////
  /// @brief @c std::_Mem_fn doppelganger that provides @c define_type().
  //////////////////////////////////////////////////////////////////////
  class wrapper
  : public std::_Maybe_unary_or_binary_function<
      _Res, const _Class*, _ArgTypes...>
  {
    using Functor = _Res (_Class::*)(_ArgTypes...) const;

    Functor __pmf;

  public:
    void define_type(typer& t)
    {
      t.base<
        std::_Maybe_unary_or_binary_function<_Res, const _Class*, _ArgTypes...>
      >(*this);

      t.member(__pmf);
    }
  };

  static wrapper&
  apply(std::_Mem_fn<_Res (_Class::*)(_ArgTypes...) const>& t) noexcept
  {
    static_assert(
      (sizeof(wrapper) ==
         sizeof(std::_Mem_fn<_Res (_Class::*)(_ArgTypes...) const>)) &&
      (std::alignment_of<wrapper>::value ==
         std::alignment_of<std::_Mem_fn<
           _Res(_Class::*)(_ArgTypes...) const>>::value),
      "Incompatible types.");
    return reinterpret_cast<wrapper&>(t);
  }
};

} // namespace stapl

#endif
