/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef KRIPKE_OPTIMIZERS_ZIP_HPP
#define KRIPKE_OPTIMIZERS_ZIP_HPP

#include <type_traits>
#include <vector>

#include "zip_traversals.hpp"
#include <stapl/skeletons/transformations/optimizers/zip.hpp>

namespace stapl {
namespace skeletons {
namespace tags {

struct lightweight_seq_execution{ };

struct kripke_sequential_execution{ };

} // namespace tags

namespace optimizers {

template <typename SkeletonTag, typename ExecutionTag>
struct optimizer;

//////////////////////////////////////////////////////////////////////
/// @brief A @c zip optimizer used for cases when the consumer needs
///        to traverse over the domain of its result. This usually happens
///        in nested computations when we passed the result of coarsened tasks
///        as the input view.
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <typename Tag, int arity>
struct optimizer<tags::zip<Tag, arity>, tags::lightweight_seq_execution>
  : public optimizer<tags::zip<Tag, arity>, tags::sequential_execution>
{
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    using type = stapl::lightweight_vector<OutputValueType>;
  };
};

//////////////////////////////////////////////////////////////////////
/// @brief A @c zip optimizer is used in case when we the
///        operator of zip represents a wavefront computation.
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <typename Tag, int arity>
struct optimizer<tags::zip<Tag, arity>, tags::kripke_sequential_execution>
{
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    using value_type = typename OutputValueType::value_type;
    using type = std::array<lightweight_vector<value_type>, 3>;
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
      kripke_zipper_iterator,
      typename std::conditional<
        pack_is_deep_sliceable<typename std::decay<Args>::type...>::value,
        kripke_zipper_slice_loop_nest,
        kripke_zipper_domain
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

#endif // KRIPKE_OPTIMIZERS_ZIP_HPP
