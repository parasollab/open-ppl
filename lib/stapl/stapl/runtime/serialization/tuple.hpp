/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_SERIALIZATION_TUPLE_HPP
#define STAPL_RUNTIME_SERIALIZATION_TUPLE_HPP

#include "typer_fwd.hpp"
#include <type_traits>

namespace std {

template<typename... Types>
class tuple;

} // namespace std


namespace stapl {

////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref define_type_provider for @c std::tuple.
///
/// @ingroup serialization
////////////////////////////////////////////////////////////////////
template<typename... Types>
struct define_type_provider<std::tuple<Types...>>
{
  template<typename... Ts>
  struct is_ref_helper
  {
    static const bool value = false;
  };

  template<typename Head, typename... Tail>
  struct is_ref_helper<Head, Tail...>
  {
    static const bool value = (std::is_reference<Head>::value ||
                               is_ref_helper<Tail...>::value);
  };

  static_assert(!is_ref_helper<Types...>::value,
                "Reference packing is not allowed.");

  //////////////////////////////////////////////////////////////////////
  /// @brief @c std::tuple doppelganger that provides @c define_type().
  //////////////////////////////////////////////////////////////////////
  class wrapper
  : public std::tuple<Types...>
  {
  private:
    //////////////////////////////////////////////////////////////////////
    /// @brief Calls @c typer::member(std::get<I>(t)) for the @c I-th element of
    ///        @p t.
    //////////////////////////////////////////////////////////////////////
    template<std::size_t I = 0>
    static typename std::enable_if<(I < sizeof...(Types)), void>::type
    apply(std::tuple<Types...>& t, typer& ct)
    {
      ct.member(std::get<I>(t));
      apply<(I + 1)>(t, ct);
    }

    //////////////////////////////////////////////////////////////////////
    /// @internal
    /// @brief Specialization of @ref apply(std::tuple<Types...>&,typer&) to end
    ///        recursion.
    //////////////////////////////////////////////////////////////////////
    template<std::size_t I = 0>
    static typename std::enable_if<(I==sizeof...(Types)), void>::type
    apply(std::tuple<Types...> const&, typer const&)
    { }

  public:
    void define_type(typer& t)
    { apply(*this, t); }
  };

  static constexpr wrapper& apply(std::tuple<Types...>& t) noexcept
  {
    return static_cast<wrapper&>(t);
  }
};

} // namespace stapl

#endif
