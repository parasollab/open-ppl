/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_BIND_HPP
#define STAPL_RUNTIME_SERIALIZATION_BIND_HPP

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

template<typename F>
struct _Weak_result_type;

template<typename Signature>
struct _Bind;

template<typename Result, typename Signature>
struct _Bind_result;

template<typename... Types>
class tuple;

} // namespace std


namespace stapl {

////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref define_type_provider for the result of
///        @c std::bind().
///
/// @warning This implementation relies on the compiler to create identical
///          layout for two classes that are written in a similar way (identical
///          members and access control). The @c static_assert calls attempt to
///          detect any possible issues if the compiler changes its behavior.
///
/// @ingroup serialization
////////////////////////////////////////////////////////////////////
template<typename F, typename... Args>
struct define_type_provider<std::_Bind<F(Args...)>>
{
  //////////////////////////////////////////////////////////////////////
  /// @brief @c std::_Bind doppelganger that provides @c define_type().
  //////////////////////////////////////////////////////////////////////
  class wrapper
  : public std::_Weak_result_type<F>
  {
    F                   _M_f;
    std::tuple<Args...> _M_bound_args;

  public:
    void define_type(typer& t)
    {
      t.base<std::_Weak_result_type<F>>(*this);
      t.member(_M_f);
      t.member(_M_bound_args);
    }
  };

  static wrapper& apply(std::_Bind<F(Args...)>& t) noexcept
  {
    static_assert(
      (sizeof(wrapper) ==
         sizeof(std::_Bind<F(Args...)>)) &&
      (std::alignment_of<wrapper>::value ==
         std::alignment_of<std::_Bind<F(Args...)>>::value),
      "Incompatible types.");
    return reinterpret_cast<wrapper&>(t);
  }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref define_type_provider() for the result of
///        @c std::bind<R>().
///
/// @warning This implementation relies on the compiler to create identical
///          layout for two classes that are written in a similar way (identical
///          members and access control). @c static_assert calls are attempting
///          to detect any possible issues if the compiler changes its behavior.
///
/// @ingroup serialization
//////////////////////////////////////////////////////////////////////
template<typename R, typename F, typename... Args>
struct define_type_provider<std::_Bind_result<R, F(Args...)>>
{
  //////////////////////////////////////////////////////////////////////
  /// @brief @c std::_Bind_result doppelganger that provides @c define_type().
  //////////////////////////////////////////////////////////////////////
  class wrapper
  {
    F                   _M_f;
    std::tuple<Args...> _M_bound_args;

  public:
    void define_type(typer& t)
    {
      t.base<std::_Weak_result_type<F>>(*this);
      t.member(_M_f);
      t.member(_M_bound_args);
    }
  };

  static wrapper& apply(std::_Bind_result<R, F(Args...)>& t) noexcept
  {
    static_assert(
      (sizeof(wrapper) ==
        sizeof(std::_Bind_result<R, F(Args...)>)) &&
      (std::alignment_of<wrapper>::value ==
        std::alignment_of<std::_Bind_result<R, F(Args...)>>::value),
      "Incompatible types.");
    return reinterpret_cast<wrapper&>(t);
  }
};

} // namespace stapl

#endif
