/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_WRAPPED_SKELETON_FWD_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_WRAPPED_SKELETON_FWD_HPP

#include <stapl/skeletons/transformations/transform_fwd.hpp>

namespace stapl {
namespace skeletons {

template <typename S, typename ExecutionTag, typename ExecutionParams,
          bool requiresTGV>
class wrapped_skeleton;

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_WRAPPED_SKELETON_FWD_HPP
