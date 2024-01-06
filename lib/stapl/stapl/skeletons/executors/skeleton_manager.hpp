/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXECUTORS_SKELETON_MANAGER_HPP
#define STAPL_SKELETONS_EXECUTORS_SKELETON_MANAGER_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/flows/view_flow.hpp>
#include <stapl/skeletons/flows/dontcare_flow.hpp>
#include <stapl/skeletons/executors/memento.hpp>
#include <stapl/skeletons/executors/spawner.hpp>

namespace stapl {
namespace skeletons {

//////////////////////////////////////////////////////////////////////
/// @brief @c skeleton_manager is the core entity of the skeleton
/// framework. It uses an environment, a set of input views, and a
/// skeleton to evaluate (spawn) a skeleton. An evaluated (spawned)
/// skeleton can be represented differently in different environments.
/// For example, a spawned skeleton in a @c taskgraph_env is a task
/// graph. A spawned skeleton in a @c graphviz_env is a GraphViz
/// representation of the dependence graph, etc.
///
/// In addition, @c skeleton_manager has another important role in
/// the skeleton framework. Imagine the case that you would like to
/// pause the spawning process of a skeleton, possibly due to the
/// limited memory that you have on your system. @c skeleton_manager
/// can do that using the famous <b>Memento</b> design skeleton. A
/// Memento design skeleton can save and restore the state of a process.
/// You can use @c record_state and @c resume to perform such operations
/// in @c skeleton_manager.
///
/// @see memento
/// @see taskgraph_env
/// @see graphviz_env
/// @see local_env
///
/// @ingroup skeletonsExecutors
//////////////////////////////////////////////////////////////////////
class skeleton_manager
{
private:
  memento m_memento_stack;
protected:
  bool m_is_done;

public:
  skeleton_manager(void);

  //////////////////////////////////////////////////////////////////////
  /// @brief This method resumes the spawning process by spawning the
  /// element in the front of the memento deque if it is not a lazy
  /// element.
  ///
  /// If all the elements of the memento double-ended queue are already
  /// resumed and there is nothing else left to spawn, the skeleton manager
  /// assumes it is done with the spawning process and will not be invoked
  /// anymore by the @c paragraph.
  //////////////////////////////////////////////////////////////////////
  void resume();

  //////////////////////////////////////////////////////////////////////
  /// @brief The @c execute method starts the spawning process of a
  /// skeleton, given a set of input, in a given environment. This method
  /// is called once per skeleton execution. It processes inputs and makes
  /// proper flows out of the inputs and sets the size of the skeletons
  /// in order to make them ready for the execution.
  ///
  /// @param skeleton the skeleton to be spawned
  /// @param env      the environment that this skeleton is going to be
  ///                 evaluated in
  /// @param views    the set of input/output data passed to the skeleton
  //////////////////////////////////////////////////////////////////////
  template <typename Skeleton, typename Env, typename ...V>
  void execute(Skeleton&& skeleton, Env env, V&... views)
  {
    spawner<Env> spnr(env, m_memento_stack);
    skeleton.set_dimensions(spnr, views...);
    spnr.spawn(std::forward<Skeleton>(skeleton), 0/*last_id*/,
               stapl::tuple<>(), stapl::tuple<>(),
               stapl::make_tuple(
                 flows::make_view_flow<V>::apply(skeleton, views)...),
               stapl::make_tuple(flows::dontcare_flow()));
  }

  void define_type(typer& t)
  {
    t.transient(m_memento_stack);
    t.member(m_is_done);
  }
};

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_EXECUTORS_SKELETON_MANAGER_HPP
