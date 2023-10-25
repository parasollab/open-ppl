/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_SCAN_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_SCAN_HPP

#include <vector>
#include <numeric>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/optimizer.hpp>

namespace stapl {
namespace skeletons {
namespace optimizers {

template <typename SkeletonTag, typename ExecutionTag>
struct optimizer;

//////////////////////////////////////////////////////////////////////
/// @brief A @c scan optimizer is used for the scan algorithm whenever
/// the inputs are local to a location. This optimizer calls
/// @c std::partial_sum to compute the results. You should notice that
/// the allocation of space for the result value takes significant
/// time for larger inputs and should be used with care.
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <typename Algorithm>
struct optimizer<tags::scan<Algorithm, tags::inclusive>,
                 tags::sequential_execution>
{
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    typedef std::vector<OutputValueType> type;
  };

  template <typename R, typename S, typename View>
  static R execute(S&& skeleton, View&& view)
  {
    std::vector<typename R::value_type> result(view.size());
    std::partial_sum(view.begin(), view.end(), result.begin(),
                     skeleton.get_op());
    return result;
  }
};

} // namespace optimizers
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_SCAN_HPP
