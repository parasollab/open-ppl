/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_SUPPORTS_STAPL_PACKING_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_SUPPORTS_STAPL_PACKING_HPP

#include "has_define_type.hpp"
#include "../serialization/typer_traits.hpp"
#include <type_traits>

namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Returns @c std::true_type if @ref typer_traits specialization exists
///        for @p T or @c T::define_type(stapl::typer&) is defined.
///
/// @related supports_stapl_packing
/// @see has_define_type, typer_traits, supports_stapl_packing
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct supports_stapl_packing_impl
: public std::integral_constant<
           bool,
           (typer_traits_specialization<T>::value || has_define_type<T>::value)
         >
{ };


////////////////////////////////////////////////////////////////////
/// @brief Returns @c std::true_type if @ref typer_traits specialization exists
///        for @p T or @c T::define_type(stapl::typer&) is defined.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct supports_stapl_packing
: public supports_stapl_packing_impl<typename std::remove_cv<T>::type>
{ };

} // namespace runtime

} // namespace stapl

#endif
