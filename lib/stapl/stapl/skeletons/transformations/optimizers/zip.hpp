/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_OPTIMIZERS_ZIP_HPP
#define STAPL_SKELETONS_OPTIMIZERS_ZIP_HPP

#include <type_traits>
#include <vector>

#include <stapl/skeletons/utility/lightweight_vector.hpp>
#include <stapl/skeletons/transformations/optimizer.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/utility/dynamic_wf.hpp>
#include <stapl/skeletons/transformations/optimizers/utils.hpp>
#include <stapl/skeletons/transformations/optimizers/zip_traversals.hpp>

#include "is_deep_sliceable.hpp"

namespace stapl {
namespace skeletons {
namespace optimizers {

template <typename SkeletonTag, typename ExecutionTag>
struct optimizer;

//////////////////////////////////////////////////////////////////////
/// @brief A @c zip optimizer is used in the cases that all views are
/// local and their traversal would be fast which improves the
/// performance. You have to notice that the inner vector creation
/// takes some time and you have to consider it before using this
/// optimizer.
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <typename Tag, int arity>
struct optimizer<tags::zip<Tag, arity>, tags::sequential_execution>
{
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    using type = std::vector<OutputValueType>;
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Sequential zip optimizer dispatches the execution to
  /// two different signatures based on the requirement of the given
  /// zip operation. If the zip operations needs the @ref paragraph_view,
  /// the dynamic specialization is called, otherwise the non_dynamic
  /// one is called.
  ///
  /// @return the result of the zip operation (can be void)
  //////////////////////////////////////////////////////////////////////
  template <typename R, typename S, typename... Args>
  static R execute(S&& skeleton, Args&&... args)
  {
    using zipper_t = typename std::conditional<
      helpers::pack_has_iterator<typename std::decay<Args>::type...>::value,
      zipper_iterator,
      typename std::conditional<
        pack_is_deep_sliceable<typename std::decay<Args>::type...>::value,
        zipper_slice_loop_nest,
        zipper_domain
      >::type
    >::type;

    using dispatcher_t = typename std::conditional<
                           std::is_base_of<
                             dynamic_wf,
                             typename std::decay<S>::type::op_type>::value,
                           typename zipper_t::dynamic,
                           typename zipper_t::non_dynamic>::type;


    return dispatcher_t::template execute<R>(
             std::forward<S>(skeleton),
             std::forward<Args>(args)...);
  }
};


} // namespace optimizers
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_OPTIMIZERS_ZIP_HPP
