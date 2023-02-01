/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_MAPPING_FUNCTION_TRAITS_HPP
#define STAPL_VIEWS_MAPPING_FUNCTION_TRAITS_HPP


namespace stapl {

BOOST_MPL_HAS_XXX_TRAIT_DEF(is_injective)
BOOST_MPL_HAS_XXX_TRAIT_DEF(is_bijective)
BOOST_MPL_HAS_XXX_TRAIT_DEF(is_surjective)

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if a mapping function is injective.
//////////////////////////////////////////////////////////////////////
template<typename MF, bool b = has_is_injective<MF>::value>
struct is_injective
  : std::false_type
{ };


template<typename MF>
struct is_injective<MF, true>
  : MF::is_injective
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if a mapping function is surjective.
//////////////////////////////////////////////////////////////////////
template<typename MF, bool b = has_is_surjective<MF>::value>
struct is_surjective
  : std::false_type
{ };


template<typename MF>
struct is_surjective<MF, true>
  : MF::is_surjective
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if a mapping function is bijective.
///        A mapping function is bijective if it reflects a true
///        is_bijective trait, or it is both injective and surjective.
//////////////////////////////////////////////////////////////////////
template<typename MF, bool b = has_is_bijective<MF>::value>
struct is_bijective
  : std::integral_constant<bool,
      is_injective<MF>::value && is_surjective<MF>::value
    >
{ };


template<typename MF>
struct is_bijective<MF, true>
  : MF::is_bijective
{ };

} // namespace stapl

#endif
