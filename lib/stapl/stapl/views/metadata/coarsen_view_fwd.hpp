/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_COARSEN_VIEW_FWD_HPP
#define STAPL_VIEWS_METADATA_COARSEN_VIEW_FWD_HPP

#include <stapl/views/type_traits/is_extended_view.hpp>
#include <stapl/views/type_traits/is_segmented_view.hpp>
#include <stapl/views/type_traits/is_trivially_coarsenable.hpp>

namespace stapl {

namespace metadata {

template<typename View>
struct extract_view;

template<typename View>
struct extract_and_restrict_view;

template<typename View>
struct extract_and_project_view;


template<typename View>
struct select_extraction_projection_policy
{
  // it's a multidimensional view and not a segmented_view
  using is_multidimensional =
    std::integral_constant<bool, dimension_traits<View>::type::value != 1 &&
      !is_segmented_view<View>::value
    >;

  // it's an extended view
  using is_extended = std::integral_constant<bool,
    is_extended_view<View>::value
  >;

  // mapping function maps view indices to container indices of same dimensions
  using has_isomorphic_mapfunc = std::integral_constant<bool,
    dimension_traits<
      typename view_impl::mapfunc_types_helper<
        typename View::map_func_type
      >::index_type
    >::type::value
    ==
    dimension_traits<
      typename view_impl::mapfunc_types_helper<
        typename View::map_func_type
      >::gid_type
    >::type::value
  >;

  // if it has the identity mapping function
  using has_identity_map_func = std::integral_constant<bool,
    is_identity<typename View::map_func_type>::type::value &&
    !is_segmented_view<View>::value
  >;

  // if it has an infinite domain or is trivially coarsenable
  using trivially_coarsenable = std::integral_constant<bool,
    !has_finite_domain<View>::value || is_trivially_coarsenable<View>::value
  >;

  // we can avoid doing projection completely if the view is trivially
  // coarsenable
  using elide_projection_statically = std::integral_constant<bool,
    trivially_coarsenable::value ||
    (is_multidimensional::value && !is_extended::value &&
      has_isomorphic_mapfunc::value)
  >;

  // we may potentially avoid doing projection, but we need a runtime check
  using elide_projection_dynamically = has_identity_map_func;

  using require_projection = std::integral_constant<bool,
    !elide_projection_dynamically::value && !elide_projection_statically::value
  >;

  using type =
    typename std::conditional<
      // if we need to project
      require_projection::value,
      extract_and_project_view<View>,

      // else
      typename std::conditional<
        // we can skip projection for sure
        elide_projection_statically::value,
        extract_view<View>,

        // else insert runtime checks
        extract_and_restrict_view<View>
      >::type
    >::type;
};

} // namespace metadata

} // namespace stapl

#endif
