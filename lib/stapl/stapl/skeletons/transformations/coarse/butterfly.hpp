/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_COARSE_BUTTERFLY_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_COARSE_BUTTERFLY_HPP

#include <stapl/utility/utility.hpp>
#include <stapl/skeletons/functional/map.hpp>
#include <stapl/skeletons/functional/butterfly.hpp>
#include <stapl/skeletons/transformations/optimizers/butterfly.hpp>
#include <stapl/skeletons/transformations/transform.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/lightweight_vector.hpp>
#include <stapl/skeletons/utility/coarse_identity_op.hpp>

namespace stapl {
namespace skeletons {
namespace butterfly_helpers {

template <typename Op>
struct coarsened_butterfly_op
  : public Op
{
public:
  using result_type = stapl::lightweight_vector<typename Op::result_type>;

  coarsened_butterfly_op(Op op)
    : Op(op)
  { }

  template <typename V1, typename V2>
  result_type
  operator()(V1 const& v1, V2 const& v2) const
  {
    result_type result;
    result.reserve(v1.size());
    std::transform(v1.begin(), v1.end(), v2.begin(),
                   std::back_inserter(result), static_cast<Op const&>(*this));
    return result;
  }

  void define_type(typer& t)
  {
    t.base<Op>(*this);
  }
};

} // namespace butterfly_helpers
} // namespace skeletons


template <typename Op>
struct identity_selector<
         skeletons::butterfly_helpers::coarsened_butterfly_op<Op>, true>
{
  using type = skeletons::coarse_identity_op<typename Op::result_type>;
};


namespace skeletons {
namespace transformations {

template <typename S, typename SkeletonTag, typename CoarseTag>
struct transform;

//////////////////////////////////////////////////////////////////////
/// @brief A coarse-grain butterfly can be created by first
/// computing a butterfly skeleton with a coarsened operator on the
/// coarsened input followed by a map of fine-grained butterfly
/// skeletons on the produced results.
///
/// @tparam S            the fine-grain butterfly scan
/// @tparam CoarseTag    a tag to specify the required specialization for
///                      coarsening
/// @tparam ExecutionTag a tag to specify the execution method used for
///                      the coarsened chunks
///
/// @see skeletonsTagsCoarse
/// @see skeletonsTagsExecution
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template<typename S, bool B, typename CoarseTag, typename ExecutionTag>
struct transform<S, tags::butterfly<B>,
                 tags::coarse<CoarseTag, ExecutionTag>>
{
  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::compose(
      skeletons::butterfly(
        butterfly_helpers::coarsened_butterfly_op<typename S::op_type>(
          skeleton.get_op())),
      skeletons::map(skeletons::wrap<ExecutionTag>(skeleton)))
  ))
};


//////////////////////////////////////////////////////////////////////
/// @brief A coarse-grain reverse-butterfly can be created by first
/// computing a map of fine-grained butterfly skeletons followed by
/// a coarse-grain reverse-butterfly with a coarsened operation
/// on the produced results.
///
/// @tparam S            the fine-grain butterfly scan
/// @tparam CoarseTag    a tag to specify the required specialization for
///                      coarsening
/// @tparam ExecutionTag a tag to specify the execution method used for
///                      the coarsened chunks
///
/// @see skeletonsTagsCoarse
/// @see skeletonsTagsExecution
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template<typename S, bool B, typename CoarseTag, typename ExecutionTag>
struct transform<S, tags::reverse_butterfly<B>,
                 tags::coarse<CoarseTag, ExecutionTag>>
{
  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::compose(
      skeletons::map(skeletons::wrap<ExecutionTag>(skeleton)),
      skeletons::reverse_butterfly(
        butterfly_helpers::coarsened_butterfly_op<typename S::op_type>(
          skeleton.get_op())))
  ))
};
} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_COARSE_BUTTERFLY_HPP

