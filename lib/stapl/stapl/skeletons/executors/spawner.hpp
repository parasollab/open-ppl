/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXECUTORS_SPAWNER_HPP
#define STAPL_SKELETONS_EXECUTORS_SPAWNER_HPP

#include <stapl/runtime/config/types.hpp>
#include <stapl/skeletons/executors/memento.hpp>
#include <stapl/paragraph/edge_container/utility.hpp>

namespace stapl {
namespace skeletons {

//////////////////////////////////////////////////////////////////////
/// @brief The spawner in the skeleton framework can be in only 3
/// possible states. Each state considers the @c spawn_element
/// requests, differently.
///
/// @li DEFAULT       calls the @c spawn_element of the environment
///                   with the given number of successors
/// @li SET_NUM_SUCCS calls @c set_num_succs of the environment for a
///                   given entity
/// @li SET_HOLD      similar to @c DEFAULT but it sets the number of
///                   successors to @c stapl::defer_spec
///
/// @ingroup skeletonsExecutorsInternal
//////////////////////////////////////////////////////////////////////
enum spawn_mode {DEFAULT, SET_NUM_SUCCS, SET_HOLD};

//////////////////////////////////////////////////////////////////////
/// @brief The spawner is used in conjunction with the
/// @c skeleton_manager to create nodes in the given environment. It
/// is the main interface between skeletons and @c skeleton_manager and
/// also the environment being used.
/// Therefore, it offers an interface for recording the state of
/// a skeleton's spawning process, and provides @c spawn_element
/// interface that would do different activities depending on the
/// @c spawn_mode that the spawner is currently set as.
///
/// @tparam Env  the environment that the spawning process should
///              happen in
///
/// @ingroup skeletonsExecutors
//////////////////////////////////////////////////////////////////////
template <typename Env>
class spawner
{
private:
  using env_type     = Env;

  env_type m_env;
  memento& m_memento_stack;
  enum spawn_mode m_spawn_mode;

public:
  spawner(env_type env, memento& memento_stack)
    : m_env(std::move(env)),
      m_memento_stack(memento_stack),
      m_spawn_mode(DEFAULT)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief This method is a simple redirection back and forth to the
  /// skeleton. It helps in tracking the spawning process in a
  /// centralized fashion, and can be further changed to perform various
  /// operations upon the invocation.
  ///
  /// @param skeleton      the skeleton to be spawned by this spawner
  /// @param lid_offset   the safe id to start the spawning from
  /// @param skeleton_size the size of skeletons
  /// @param coord        the coordinate of the skeleton
  /// @param in           the input flow of the skeleton
  /// @param out          the output flow of the skeleton
  /// @param cur_stage    the current stage/state of the skeleton
  ///
  /// @return true        if spawning of the skeleton is finished
  //////////////////////////////////////////////////////////////////////
  template <typename Skeleton, typename Coord, typename In, typename Out>
  bool spawn(Skeleton& skeleton,
             std::size_t lid_offset,
             Coord const& skeleton_size, Coord const& coord,
             In const& in, Out const& out,
             std::size_t cur_stage = 0)
  {
    m_env.pre_spawn(
      skeleton, lid_offset, skeleton_size, coord, in, out, cur_stage);

    bool is_complete = skeleton.spawn(*this, lid_offset, skeleton_size, coord,
                                      in, out,
                                      cur_stage);
    m_env.post_spawn(
      skeleton, lid_offset, skeleton_size, coord, in, out, cur_stage);
    return is_complete;
  }

  env_type get_env()
  {
    return m_env;
  }

  memento& get_memento_stack()
  {
    return m_memento_stack;
  }

  void set_spawn_mode(enum spawn_mode cur_spawn_mode)
  {
    m_spawn_mode = cur_spawn_mode;
  }

  enum spawn_mode spawn_mode() const
  {
    return m_spawn_mode;
  }

  std::size_t get_num_PEs() const
  {
    return m_env.get_num_PEs();
  }

  runtime::location_id get_PE_id() const
  {
    return m_env.get_PE_id();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief @copybrief memento::record_state
  ///
  /// @param skeleton       the skeleton for which the state should be
  ///                      recorded
  /// @param lid_offset    the id offset of the skeleton after pausing
  /// @param skeleton_size  the size of the given skeleton
  /// @param coord         the coordinate of the given skeleton
  /// @param in            the input flow that should be passed to the
  ///                      skeleton upon continuation
  /// @param out           the output flow that should be passed to the
  ///                      skeleton upon continuation
  /// @param cur_stage     current stage of the paused skeleton. It is
  ///                      needed to determine the state upon resuming
  /// @param is_lazy       if this skeleton can stay untouched in the
  ///                      memento
  /// @param front         if this state should be inserted in the front
  ///                      of the memento double-ended queue
  ///
  /// @see skeleton_manager::record_state
  //////////////////////////////////////////////////////////////////////
  template <typename Skeleton,
            typename Coord,
            typename In, typename Out>
  void record_state(Skeleton& skeleton, std::size_t lid_offset,
                    Coord& skeleton_size, Coord& coord,
                    In const& in, Out const& out,
                    std::size_t cur_stage,
                    bool is_lazy = false, bool front = false)
  {
    m_memento_stack.record_state(*this,
                                 skeleton,
                                 lid_offset,
                                 skeleton_size, coord,
                                 in, out,
                                 cur_stage, is_lazy, front);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief When a spawner is asked to spawn an element, it checks
  /// the @c spawn_mode that it is now at. This is regardless of which
  /// environment the @c spawn_element request is going to be evaluated
  /// in. It would be then the responsibility of an environment to
  /// perform the appropriate action on such @c spawn_element or
  /// @c set_num_succs requests.
  ///
  /// @param tid        the unique element id that is going to be assigned to
  ///                   this element
  /// @param result_id  the result id of the task if it has any
  /// @param wf         the work function to be executed on the given inputs
  /// @param mapper     the output to output mapper for mapping the results
  ///                   of the child paragraph to current paragraph
  /// @param in         the input @c producer_info to the workfunction
  //////////////////////////////////////////////////////////////////////
  template <bool isResult, typename WF, typename Mapper, typename... In>
  void spawn_element(std::size_t tid, std::size_t result_id, WF&& wf,
                     Mapper&& mapper, std::size_t no_succs, In&&... in)
  {
    switch (m_spawn_mode) {
      case DEFAULT:
        this->m_env.template spawn_element<isResult>(
          tid, result_id, std::forward<WF>(wf), std::forward<Mapper>(mapper),
          no_succs, std::forward<In>(in)...);
        break;
      case SET_HOLD:
        this->m_env.template spawn_element<isResult>(
          tid, result_id, std::forward<WF>(wf), std::forward<Mapper>(mapper),
          stapl::defer_spec, std::forward<In>(in)...);
        break;
      case SET_NUM_SUCCS:
        this->m_env.set_num_succs(tid, no_succs);
        break;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief When a spawner is asked to spawn an element, it checks
  /// the @c spawn_mode that is now at. This is regardless of which
  /// environment the @c spawn_element request is going to be evaluated
  /// in. It would be then the responsibility of an environment to
  /// perform the appropriate action on such @c spawn_element or
  /// @c set_num_succs requests.
  ///
  /// @param tid        the unique element id that is going to be
  ///                   assigned to this element
  /// @param result_id  the result id of the task if it has any
  /// @param wf         the work function to be executed on the given inputs
  /// @param in         the input @c producer_info to the workfunction
  //////////////////////////////////////////////////////////////////////
  template <bool isResult, typename WF, typename Mapper, typename... In>
  void spawn_element(std::size_t tid, std::size_t result_id,
                     std::vector<std::size_t> const& notifications, WF&& wf,
                     Mapper&& mapper, std::size_t no_succs, In&&... in)
  {
    switch (m_spawn_mode) {
      case DEFAULT:
        this->m_env.template spawn_element<isResult>(
          tid, result_id, notifications, std::forward<WF>(wf),
          std::forward<Mapper>(mapper), no_succs, std::forward<In>(in)...);
        break;
      // we should not get to the next two cases, maybe we should assert?
      case SET_HOLD:
        this->m_env.template spawn_element<isResult>(
          tid, result_id, std::forward<WF>(wf), std::forward<Mapper>(mapper),
          stapl::defer_spec, std::forward<In>(in)...);
        break;
      case SET_NUM_SUCCS:
        this->m_env.set_num_succs(tid, no_succs);
        break;
    }
  }
};

} // namespace skeleton
} // namespace stapl

#endif // STAPL_SKELETONS_EXECUTORS_SPAWNER_HPP
