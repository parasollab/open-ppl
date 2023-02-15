/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_REDUCE_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_REDUCE_HPP

#include <numeric>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/optimizer.hpp>

namespace stapl {
namespace skeletons {
namespace optimizers {

template <typename SkeletonTag, typename ExecutionTag>
struct optimizer;

//////////////////////////////////////////////////////////////////////
/// @brief A @c reduce optimizer is used whenever the inputs are local
/// to a nested execution which improves the performance.
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <typename Align>
struct optimizer<tags::reduce<Align>, tags::sequential_execution>
{
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    using type = OutputValueType;
  };

  template <typename R, typename S, typename View1>
  static R execute(S&& skeleton, View1&& view1)
  {
    using value_t = typename std::decay<View1>::type::value_type;
    stapl_assert(view1.size() > 0, "reduce_wf found empty_view");
    return std::accumulate(++(view1.begin()), view1.end(),
                           (value_t)*(view1.begin()),
                           skeleton.get_op());
  }
};

} // namespace optimizers
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_REDUCE_HPP
