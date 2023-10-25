/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_IS_EXTENDED_VIEW_HPP
#define STAPL_VIEWS_TYPE_TRAITS_IS_EXTENDED_VIEW_HPP

#include <stapl/views/view_traits.hpp>
#include <type_traits>

namespace stapl {

namespace detail {

template <size_t FixedIndex, typename T, typename Sequence>
struct rewire_dimension;

template<typename MappingFunction>
struct is_rewire_dimension
  : public std::false_type
{ };

template <size_t Fixed, typename T, typename Sequence>
struct is_rewire_dimension<rewire_dimension<Fixed, T, Sequence>>
  : public std::true_type
{ };

}

//////////////////////////////////////////////////////////////////////
/// @brief Type checker to determine if the given @p View type is an
///        instantiation of extended_view.
//////////////////////////////////////////////////////////////////////
template <typename View>
struct is_extended_view
  : std::integral_constant<bool,
      detail::is_rewire_dimension<
        typename view_traits<View>::map_function>::value
    >
{ };

} //namespace stapl

#endif // STAPL_VIEWS_TYPE_TRAITS_IS_EXTENDED_VIEW_HPP
