/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXECUTORS_PARAGRAPH_SKELETON_MANAGER_HPP
#define STAPL_SKELETONS_EXECUTORS_PARAGRAPH_SKELETON_MANAGER_HPP

#include "skeleton_manager.hpp"
#include <stapl/skeletons/environments/taskgraph_env.hpp>
#include <stapl/skeletons/environments/combined_env.hpp>
#include <stapl/paragraph/incremental_wf.hpp>
#include <stapl/paragraph/factory_wf.hpp>
#include "paragraph_skeleton_manager_fwd.hpp"
#include "location_mapper.hpp"


namespace stapl {
namespace skeletons {

namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This is a wrapper over the core @c skeleton_manager
/// that allows the creating of tasks in STAPL task graphs.
///
/// @tparam Skeleton        the skeleton to be executed.
/// @tparam ExecutionParams extra configurations to customize the execution
///                         of the given skeleton.
///
/// @see skeleton_manager
/// @see combined_env
/// @see graphviz_env
/// @see local_env
///
/// @ingroup skeletonsExecutors
//////////////////////////////////////////////////////////////////////
template <typename Skeleton, typename ExecutionParams>
class paragraph_skeleton_manager
  : public skeleton_manager,
    public incremental_wf,
    public factory_wf,
    public ExecutionParams
{
public:
  using coarsener_type      = typename ExecutionParams::coarsener_type;
  using extra_env_type      = typename ExecutionParams::extra_env_type;
  using result_type         = typename ExecutionParams::result_type;
  using scheduler_type      = typename ExecutionParams::scheduler_type;
  using skeleton_tag_type   = typename Skeleton::skeleton_tag_type;

  using span_type           = void;
  using task_id_mapper_type = stapl::use_default;

  template <typename In>
  using out_port_type = typename Skeleton::template out_port_type<In>;

private:
  /// @brief checks if it is the first time that @c PARAGRAPH is calling this
  /// factory.
  bool     m_is_first_iter;
  Skeleton m_skeleton;

public:
  paragraph_skeleton_manager(
    Skeleton const& skeleton,
    ExecutionParams ep)
    : ExecutionParams(std::move(ep)),
      m_is_first_iter(true),
      m_skeleton(skeleton)
  { }

  coarsener_type get_coarsener() const
  { return ExecutionParams::coarsener(); }

  scheduler_type get_scheduler(void) const
  { return ExecutionParams::scheduler(); }

  Skeleton get_skeleton(void) const
  { return m_skeleton; }

  //////////////////////////////////////////////////////////////////////
  /// @brief A simple optimization to avoid the @c empty_env from being
  /// called during the spawning process.
  //////////////////////////////////////////////////////////////////////
  template <typename TGV>
  taskgraph_env<TGV>
  create_envs(TGV tgv, std::true_type)
  {
    return taskgraph_env<TGV>(tgv);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief If the extra environment passed to the paragraph skeleton
  /// manager is not an empty environment, it should be bundled with
  /// the taskgraph environment before starting the spawning process.
  //////////////////////////////////////////////////////////////////////
  template <typename TGV>
  combined_env<extra_env_type, taskgraph_env<TGV>>
  create_envs(TGV tgv, std::false_type)
  {
    return make_combined_env(
             ExecutionParams::extra_env(), taskgraph_env<TGV>(tgv));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief This function combines a @c taskgraph_env with a
  /// user-provided environment (if any) and then executes the skeleton
  /// in the combined environment.
  ///
  /// This method will be called by @c PARAGRAPH until this factory
  /// specifies that it is done with task creation (@c m_is_done).
  /// Upon each invocation it asks the @c skeleton_manager to continue
  /// spawning the enclosed skeleton.
  ///
  /// @param tgv      task graph view is passed to this method by
  ///                 @c PARAGRAPH using this argument
  /// @param view     set of input views to the algorithm
  //////////////////////////////////////////////////////////////////////
  template <typename TGV, typename ...V>
  void operator()(TGV tgv, V&&... view)
  {
    if (this->m_is_first_iter) {
      this->m_is_first_iter = false;

      auto envs = create_envs(tgv,
                              std::is_same<extra_env_type, stapl::use_default>());
      envs.init_location_info(tgv.graph().get_num_locations(),
                              tgv.graph().get_location_id());
      this->execute(m_skeleton, envs, view...);
    } else {
      stapl_assert(!this->m_is_done,
        "paragraph skeleton manager was reinvoked after it was done");
      this->resume();
    }
  }

  bool finished(void) const
  {
    return this->m_is_done;
  }

  template<typename View, typename... Views>
  auto get_task_id_mapper(View const& view, Views const&... views) const
   -> decltype(task_id_mapper(view))
  {
    return task_id_mapper(view);
  }

  void define_type(typer& t)
  {
    t.base<skeleton_manager>(*this);
    t.base<incremental_wf>(*this);
    t.base<factory_wf>(*this);
    t.base<ExecutionParams>(*this);
    t.member(m_is_first_iter);
    t.member(m_skeleton);
  }
};

} // namespace skeletons_impl


//////////////////////////////////////////////////////////////////////
/// @brief Creates a @c paragraph_skeleton_manager using the given
/// parameters. The default values for the arguments are provided in
/// paragraph_skeleton_manager_fwd.hpp.
///
/// @param skeleton         the skeleton to be spawned.
/// @param execution_params customize the execution process
///                         (@see execution_params.hpp)
///
/// @return a skeleton manager over the given skeleton
///
/// @ingroup skeletonsExecutors
//////////////////////////////////////////////////////////////////////
template <typename S, typename ExecutionParams>
skeletons_impl::paragraph_skeleton_manager<
  typename std::decay<S>::type,
  typename std::decay<ExecutionParams>::type>
make_paragraph_skeleton_manager(S&& skeleton, ExecutionParams&& ep)
{
  return skeletons_impl::paragraph_skeleton_manager<
           typename std::decay<S>::type,
           typename std::decay<ExecutionParams>::type
         >(std::forward<S>(skeleton), std::forward<ExecutionParams>(ep));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_EXECUTORS_PARAGRAPH_SKELETON_MANAGER_HPP
