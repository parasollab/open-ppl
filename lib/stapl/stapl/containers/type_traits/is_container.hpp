/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_TYPE_TRAITS_IS_CONTAINER_HPP
#define STAPL_CONTAINERS_TYPE_TRAITS_IS_CONTAINER_HPP

#include <boost/mpl/has_xxx.hpp>

namespace stapl {

BOOST_MPL_HAS_XXX_TRAIT_DEF(distribution_type)

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if a given type is a STAPL container.
/// The criteria for modeling a container is based on the existence
/// of a nested distribution_type in T.
///
/// @tparam T The type in question
//////////////////////////////////////////////////////////////////////
template<typename T>
struct is_container
 : public has_distribution_type<T>
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if a given type is a STAPL container.
/// The criteria for modeling a container is based on the existence
/// of a nested distribution_type in T.
///
/// This metafunction is used in @ref mix_view to determine when the element
/// type of a container is a nested container.
///
/// @tparam T The type in question
//////////////////////////////////////////////////////////////////////
template<typename T>
struct is_nested_container
 : public has_distribution_type<T>
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if a given type is a STAPL container.
/// This is a specialization used for map elements. The criteria for
/// modeling a container is based on the existence of a nested
/// distribution_type in T.
///
/// This metafunction is used in @ref mix_view to determine when the element
/// type of a container is a nested container.
///
/// @tparam T The type in question
//////////////////////////////////////////////////////////////////////
template<typename T, typename U>
struct is_nested_container<std::pair<T, U>>
 : public has_distribution_type<U>
{ };

} // namespace stapl

#endif
