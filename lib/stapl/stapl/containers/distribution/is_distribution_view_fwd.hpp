/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_IS_DISTRIBUTION_VIEW_FWD_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_IS_DISTRIBUTION_VIEW_FWD_HPP

#include <stapl/views/type_traits/is_view.hpp>

namespace stapl {

template <typename V, bool is_vw = is_view<V>::value>
struct is_distribution_view;

template <typename DistributionView, typename PartitionInfoContainer>
struct view_based_partition;

template <typename DistributionView>
struct view_based_mapper;

template <typename T>
struct is_view_based;

template <typename T>
struct is_arbitrary_view_based;

namespace detail {

BOOST_MPL_HAS_XXX_TRAIT_DEF(is_composed_dist_spec)

}

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_IS_DISTRIBUTION_VIEW_FWD_HPP
