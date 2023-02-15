/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_IS_INVERTIBLE_VIEW_HPP
#define STAPL_VIEWS_TYPE_TRAITS_IS_INVERTIBLE_VIEW_HPP

#include <boost/mpl/has_xxx.hpp>
#include <stapl/views/metadata/infinite_helpers.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if an object has a nested trait inverse
//////////////////////////////////////////////////////////////////////
BOOST_MPL_HAS_XXX_TRAIT_DEF(inverse)

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to check if the domain of the view and its underlying
/// container are the same type.
///
/// The specialization of the class is needed to prevent instantiation of
/// view_traits on std container types.
//////////////////////////////////////////////////////////////////////
template <typename View, typename Container,
          bool has_dom = has_domain_type<Container>::value>
struct has_same_domain
  : public std::false_type
{ };

template <typename View, typename Container>
struct has_same_domain<View, Container, true>
  : public std::is_same<
      typename view_traits<View>::domain_type,
      typename view_traits<Container>::domain_type
    >
{ };

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if a view has an inverse function
///        defined for its mapping function and a finite domain.
//////////////////////////////////////////////////////////////////////
template <typename View>
struct is_invertible_view
  : std::integral_constant<bool,
      // the mapping function has an inverse defined
      detail::has_inverse<typename view_traits<View>::map_function>::value &&
      // the view's domain is finite
      has_finite_domain<View>::value &&
      !std::is_same<typename view_traits<View>::container,
        std::vector<typename view_traits<View>::value_type>>::value &&
      // the view and its container have the same domain
      detail::has_same_domain<View,
        typename view_traits<View>::container>::value
    >
{ };

} //namespace stapl

#endif /* STAPL_VIEWS_TYPE_TRAITS_IS_INVERTIBLE_VIEW_HPP */
