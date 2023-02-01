/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_BUTTERFLY_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_BUTTERFLY_HPP

#include <numeric>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/optimizer.hpp>
#include <stapl/skeletons/utility/lightweight_vector.hpp>

namespace stapl {
namespace skeletons {
namespace optimizers {

template <typename SkeletonTag, typename ExecutionTag>
struct optimizer;

//////////////////////////////////////////////////////////////////////
/// @brief A @c butterfly optimizer is used whenever the inputs are local.
/// The knowledge of the inputs being local allows the usage of
/// @c std::accumulate for a faster execution.
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <bool B>
struct optimizer<tags::butterfly<B>, tags::sequential_execution>
{
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    using type = stapl::lightweight_vector<OutputValueType>;
  };

  template <typename R, typename S, typename View1>
  static R
  execute(S&& skeleton, View1 const& view1)
  {
    stapl_assert(view1.size() > 0,
                 "butterfly commutative optimizer found empty view");
    // use the first element of the view as the initial value for accumulate
    auto sum = std::accumulate(++(view1.begin()), view1.end(),
                               (typename View1::value_type)*(view1.begin()),
                               skeleton.get_op());

    R result_v;
    result_v.reserve(view1.size());
    for (std::size_t i = 0; i < view1.size(); ++i) {
      result_v.push_back(sum);
    }

    return result_v;
  }
};


template <bool B>
struct optimizer<tags::reverse_butterfly<B>, tags::sequential_execution>
  : public optimizer<tags::butterfly<B>, tags::sequential_execution>
{ };

} // namespace optimizers
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_BUTTERFLY_HPP
