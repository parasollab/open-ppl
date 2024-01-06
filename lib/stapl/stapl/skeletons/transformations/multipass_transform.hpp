/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
#include <stapl/skeletons/utility/tags.hpp>

namespace stapl {
namespace skeletons {
namespace transformations {

template <typename S, typename SkeletonTag, typename CoarseTag>
struct transform;

template<class S, typename SkeletonTag>
struct transform<S, SkeletonTag, tags::multipass_transform<>>
{
  static S const& call(S const& skeleton)
  {
    return skeleton;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Apply a sequence of transformations on a skeleton identified
///        by the tags in the @see multipass_transform tag.
//////////////////////////////////////////////////////////////////////
template <typename S, typename SkeletonTag, typename Pass, typename... Passes>
struct transform<S, SkeletonTag, tags::multipass_transform<Pass, Passes...>>
{
private:
  using after_pass = decltype(skeletons::transform<Pass>(std::declval<S>()));
  using base_after = typename after_pass::base_type;

public:
  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::transform<tags::multipass_transform<Passes...>, base_after>(
      skeletons::transform<Pass>(skeleton))
  ))
};

} // namespace transformations
} // namespace skeletons
} // namespace stapl
