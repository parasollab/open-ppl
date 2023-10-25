/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXECUTORS_EXECUTE_HPP
#define STAPL_SKELETONS_EXECUTORS_EXECUTE_HPP

#include <stapl/skeletons/executors/execute_fwd.hpp>
#include <stapl/skeletons/executors/execution_params.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/skeletons/executors/paragraph_skeleton_manager.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @copybrief skeletons::execute
///
/// Special case for blocking paragraphs
/// @ingroup skeletonsExecutors
//////////////////////////////////////////////////////////////////////
template <typename ExecutionParams, typename S, typename... Views>
typename ExecutionParams::result_type execute(std::false_type,
                                              ExecutionParams const& ep,
                                              S&& skeleton, Views&&... views)
{
  using scheduler_type = typename ExecutionParams::scheduler_type;

  auto pmg = make_paragraph_skeleton_manager(std::forward<S>(skeleton), ep);

  return paragraph<
           scheduler_type, decltype(pmg),
           typename std::decay<Views>::type...
         >(pmg, std::forward<Views>(views)..., ep.scheduler())();
}

//////////////////////////////////////////////////////////////////////
/// @copybrief skeletons::execute
///
/// Special case for non-blocking paragraphs
/// @ingroup skeletonsExecutors
//////////////////////////////////////////////////////////////////////
template <typename ExecutionParams, typename S, typename... Views>
typename ExecutionParams::result_type execute(std::true_type,
                                              ExecutionParams const& ep,
                                              S&& skeleton, Views&&... views) {
  using scheduler_type = typename ExecutionParams::scheduler_type;

  auto pmg = make_paragraph_skeleton_manager(std::forward<S>(skeleton), ep);

  using pmg_t = paragraph<scheduler_type, decltype(pmg),
                          typename std::decay<Views>::type...>;
  pmg_t* pg = new pmg_t(pmg, std::forward<Views>(views)..., ep.scheduler());
  (*pg)(0);
}

}  // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Given a @c skeleton, and a set of views this method spawns
/// the skeleton in a @c taskgraph_env.
///
/// The @c execution_params define various configurations for execution
/// including coarsening method and the scheduler to be used and also
/// determine whether the execution should be non-blocking. In addition,
/// the @c execution_params determine if the execution would return
/// a value after finishing.
///
/// @param execution_params customize the execution process
///                         (@see execution_params.hpp)
/// @param skeleton         the skeleton to be spawned
/// @param views...         the set of views to be used for spawning
///
/// @return a reference to the result only if @c execution_params::result_type
///         is not void.
///
/// @ingroup skeletonsExecutors
//////////////////////////////////////////////////////////////////////
template <typename ExecutionParams, typename S, typename... Views>
typename ExecutionParams::result_type
execute(
  ExecutionParams const& execution_params, S&& skeleton, Views&&... views)
{
  return skeletons_impl::execute(
    std::integral_constant<bool, ExecutionParams::is_blocking>(),
    execution_params, std::forward<S>(skeleton), std::forward<Views>(views)...);
}

}  // namespace skeletons
}  // namespace stapl

#endif  // STAPL_SKELETONS_EXECUTORS_EXECUTE_HPP
