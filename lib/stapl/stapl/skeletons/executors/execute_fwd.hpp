/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXECUTORS_EXECUTE_FWD_HPP
#define STAPL_SKELETONS_EXECUTORS_EXECUTE_FWD_HPP

namespace stapl {
namespace skeletons {

// Refer to execute.hpp for implementation of the following function.
template <typename ExecutionParams, typename S, typename... Views>
typename ExecutionParams::result_type execute(
    ExecutionParams const& execution_params, S&& skeleton, Views&&... views);

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_EXECUTORS_EXECUTE_FWD_HPP


