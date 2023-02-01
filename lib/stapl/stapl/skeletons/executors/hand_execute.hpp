/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXECUTORS_HAND_EXECUTE_HPP
#define STAPL_SKELETONS_EXECUTORS_HAND_EXECUTE_HPP

#include <stapl/skeletons/executors/execution_params.hpp>
#include <stapl/utility/distributed_value.hpp>
#include <stapl/algorithms/identity_value.hpp>
#include <stapl/views/metadata/base_container_range.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

template<int arity, typename ZipOp, typename RedOp>
struct zip_reduce;

//////////////////////////////////////////////////////////////////////
/// @brief  Execute a skeleton in place with a set of views.
///
/// @tparam Skeleton The skeleton to execute
/// @tparam Views The views to use as the source of data
//////////////////////////////////////////////////////////////////////
template<typename Skeleton, typename... Views>
struct hand_execute_impl
{
  static_assert(sizeof(Skeleton) == 0,
                "No implementation for the provided skeleton");
};


//////////////////////////////////////////////////////////////////////
/// @brief  Execute a zip_reduce with one view in place.
///
/// This implementation requires that identity_value is defined for the
/// reduction operator. This is for two reasons:
///   1. This method is SPMD and thus there needs to be a way to incorporate
///      an empty location's result without affecting the globally reduced
///      value.
///   2. It is not clear how to have a preamble that only applies the zip
///      operator to the first element, and then skips the first element
///      for the main loop, since there are nested loops.
///
/// @tparam ZipOp The zip operator of the skeleton
/// @tparam ReduceOp The reduction operator of the skeleton
/// @tparam View The single view to use as the source of data
//////////////////////////////////////////////////////////////////////
template <typename ZipOp, typename ReduceOp, typename View>
struct hand_execute_impl<stapl::skeletons::skeletons_impl::
                           zip_reduce<1, ZipOp, ReduceOp>,
                         View>
{
private:
  using value_type = typename View::value_type;
  using zip_value_type
    = decltype(std::declval<ZipOp>()(std::declval<value_type>()));

public:
  using result_type = decltype(std::declval<ReduceOp>()(
    std::declval<zip_value_type>(), std::declval<zip_value_type>()));

  template<typename Skeleton, typename V>
  static result_type execute(Skeleton&& skel, V&& view)
  {
    const bool is_trivially_coarsenable = has_identity_mf<View>::type::value
      && view.domain().is_same_container_domain();

    // Compute the local zip reduce for the values on this locations
    result_type running = is_trivially_coarsenable
      ? local_execute_no_coarsen(std::forward<Skeleton>(skel),
                                 std::forward<V>(view))
      : local_execute_coarsen(
          std::forward<Skeleton>(skel),
          std::get<0>(stapl::default_coarsener()(std::make_tuple(view))));

    // Perform the global reduction using RMIs
    distributed_value<result_type> dv(running);
    auto fut = dv.reduce(skel.get_reduce_op());

    // Wait until the zip / reduce operators have finished their RMIs
    rmi_fence();

    return fut.get();
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Perform the local map_reduce using the view's containers
  /// local base containers
  ///
  /// @param skel The zip_reduce skeleton
  /// @param view The single view to use as the source of data
  //////////////////////////////////////////////////////////////////////
  template<typename Skeleton, typename V>
  static result_type local_execute_no_coarsen(Skeleton&& skel, V&& view)
  {
    auto const zip_op = skel.get_zip_op();
    auto const reduce_op = skel.get_reduce_op();

    result_type running = identity_value<ReduceOp, result_type>::value();

    for (auto& bc : metadata::make_base_container_range(std::forward<V>(view)))
      for (auto elem : bc)
        running = reduce_op(running, zip_op((std::move(elem))));

    return running;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Perform the local map_reduce using the view's metadata elements
  ///
  /// @param skel The zip_reduce skeleton
  /// @param view The single view to use as the source of data
  //////////////////////////////////////////////////////////////////////
  template <typename Skeleton, typename Coarsened>
  static result_type local_execute_coarsen(Skeleton&& skel,
                                           Coarsened&& coarsened)
  {
    auto const zip_op = skel.get_zip_op();
    auto const reduce_op = skel.get_reduce_op();

    result_type running = identity_value<ReduceOp, result_type>::value();

    for (auto md : coarsened)
    {
      for (auto v : *md.get_component())
      {
        running = reduce_op(running, zip_op((std::move(v))));
      }
    }

    return running;
  }
};

}  // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Given a @c skeleton, and a set of views this method executes
/// the skeleton in-place without an explicit task graph.
///
/// @param skeleton         the skeleton to be spawned
/// @param views...         the set of views to be used for spawning
///
/// @ingroup skeletonsExecutors
//////////////////////////////////////////////////////////////////////
template <typename S, typename... Views>
typename skeletons_impl::hand_execute_impl<
  typename std::decay<S>::type,
  typename std::decay<Views>::type...>::result_type
hand_execute(S&& skeleton, Views&&... views)
{

  using executor_type
    = skeletons_impl::hand_execute_impl<typename std::decay<S>::type,
                                        typename std::decay<Views>::type...>;

  return executor_type::execute(std::forward<S>(skeleton),
                                std::forward<Views>(views)...);
}

}  // namespace skeletons
}  // namespace stapl

#endif
