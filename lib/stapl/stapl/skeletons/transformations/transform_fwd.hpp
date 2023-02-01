/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_TRANSFORM_FWD_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_TRANSFORM_FWD_HPP

namespace stapl {
namespace skeletons {
namespace transformations {

//////////////////////////////////////////////////////////////////////
/// @brief In order to define a new transformation, you need to define
/// a partial template specialization of this class. Since there is
/// no default transformation, this class is only declared here.
///
/// @tparam S            the skeleton to be transformed
/// @tparam SkeletonTag  should be used for partial specialization
/// @tparam TransformTag determines the type of transformation to be
///                      applied
//////////////////////////////////////////////////////////////////////
template <typename S, typename SkeletonTag, typename TransformTag>
struct transform;

} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_TRANSFORM_FWD_HPP
