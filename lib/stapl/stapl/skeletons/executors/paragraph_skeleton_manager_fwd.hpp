/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXECUTORS_PARAGRAPH_SKELETON_MANAGER_FWD_HPP
#define STAPL_SKELETONS_EXECUTORS_PARAGRAPH_SKELETON_MANAGER_FWD_HPP

#include <type_traits>
#include <stapl/utility/use_default.hpp>

namespace stapl {
namespace skeletons {

namespace skeletons_impl {

template <typename Skeleton, typename ExecutionParams>
class paragraph_skeleton_manager;

} // namespace skeletons_impl

template <typename S, typename ExecutionParams>
skeletons_impl::paragraph_skeleton_manager<
  typename std::decay<S>::type,
  typename std::decay<ExecutionParams>::type>
make_paragraph_skeleton_manager(S&& skeleton,
                                ExecutionParams&& extra_env);

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_EXECUTORS_PARAGRAPH_SKELETON_MANAGER_FWD_HPP
