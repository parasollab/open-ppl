/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXECUTORS_MEMENTO_HPP
#define STAPL_SKELETONS_EXECUTORS_MEMENTO_HPP

#include <memory>
#include <boost/intrusive/slist.hpp>

namespace stapl {
namespace skeletons {

namespace skeletons_impl {

struct memento_element_base
  : public boost::intrusive::slist_base_hook<>
{
  virtual ~memento_element_base() = default;
  virtual void operator()() = 0;
  virtual bool is_lazy() = 0;
  virtual void set_is_lazy(bool) = 0;
};


//////////////////////////////////////////////////////////////////////
/// @brief a memento element is used in memento in order to reduce the
/// compilation time by avoiding boost::bind.
///
/// A memento_element basically stores the spawning state of a skeleton
/// and allows resuming of the spawning process at a later time.
///
/// @tparam Spawner the spawner which was originally spawning the given
///                 skeleton.
/// @tparam S       the skeleton which was paused
/// @tparam Coord   the coordinate information of the paused spawning
///                 process.
/// @tparam In      the input flow to the paused spawning process.
/// @tparam Out     the output flow to the paused spawning process.
///
/// @ingroup skeletonsExecutorsInternal
//////////////////////////////////////////////////////////////////////
template <typename Spawner, typename S, typename Coord,
          typename In, typename Out>
struct memento_element
  : public memento_element_base
{
private:
  Spawner      m_spawner;
  S&           m_skeleton;
  std::size_t  m_lid_offset;
  Coord        m_skeleton_size;
  Coord        m_coord;
  In           m_in;
  Out          m_out;
  std::size_t  m_cur_stage;
  bool         m_is_lazy;

public:
  memento_element(Spawner      spawner,
                  S&           skeleton,
                  std::size_t  lid_offset,
                  Coord const& skeleton_size,
                  Coord const& coord,
                  In const&    in,
                  Out const&   out,
                  std::size_t  cur_stage)
    : m_spawner(spawner),
      m_skeleton(skeleton),
      m_lid_offset(lid_offset),
      m_skeleton_size(skeleton_size),
      m_coord(coord),
      m_in(in),
      m_out(out),
      m_cur_stage(cur_stage),
      m_is_lazy(false)
  { }

  void operator()() override
  {
    m_spawner.spawn(m_skeleton, m_lid_offset,
                    m_skeleton_size, m_coord,
                    m_in, m_out, m_cur_stage);
  }

  void set_is_lazy(bool is_lazy) override
  {
    m_is_lazy = is_lazy;
  }

  bool is_lazy() override
  {
    return m_is_lazy;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object implementing the disposer concept defined by
///   Boost.Intrusive.
///
/// Intrusive containers do not manage the lifetime of the objects they
/// contain.  This is left to the container's user to manage.  The disposer
/// callback functor allows the user to do this during container operations
/// where object deletion may be desired (i.e., clear()).  In the version
/// context, this disposer is a simple delete call.
///
/// Function operator signatures restricted to @p memento_element and
/// @p memento_element_base to guard against inadvertent use in other
/// contexts.
///
/// @ingroup skeletonsExecutorsInternal
//////////////////////////////////////////////////////////////////////
struct memento_element_disposer
{
  void operator()(memento_element_base* entry_ptr) const
  {
    delete entry_ptr;
  }

  template <typename Spawner, typename S, typename Coord,
            typename In, typename Out>
  void operator()(memento_element<Spawner, S, Coord, In, Out>* entry_ptr) const
  {
    delete entry_ptr;
  }
};
}

//////////////////////////////////////////////////////////////////////
/// @brief @c memento is following Memento design skeleton and is
/// responsible for keeping track of not-yet-spawned skeletons.
/// Skeletons can ask a @c spawner to store their current state. The
/// @c spawner in turn asks its @c skeleton_manager to store the state.
///
/// @see skeleton_manager::record_state
/// @see skeleton_manager::resume
///
/// @ingroup skeletonsExecutors
//////////////////////////////////////////////////////////////////////
class memento
{
public:
  using element_type = skeletons_impl::memento_element_base;
private:
  using internal_stack_t = boost::intrusive::slist<
                             element_type,
                             boost::intrusive::constant_time_size<false>,
                             boost::intrusive::cache_last<true>>;

  std::shared_ptr<internal_stack_t> callbacks;
public:
  memento();

  //////////////////////////////////////////////////////////////////////
  /// @brief the front element of the memento stack at times needs to
  /// stays untouched until some criteria is met. The entities that can
  /// remain untouched are called lazy. This method checks if the
  /// element is lazy.
  ///
  /// @return true only if the element on top of the memento stack is
  ///              lazy
  //////////////////////////////////////////////////////////////////////
  bool front_is_lazy();

  std::size_t size() const;

  bool is_empty();

  void pop();

  //////////////////////////////////////////////////////////////////////
  /// @brief This method records the current state of a partially
  /// spawned skeleton in the memento double-ended queue of the
  /// @c skeleton_manager
  ///
  /// @param spawner       the spawner used for spawning the enclosed
  ///                      skeleton before pausing
  /// @param skeleton       the skeleton for which the state should be
  ///                      recorded
  /// @param lid_offset    the id offset of the skeleton after pausing
  /// @param skeleton_size  the size of the enclosed skeleton
  /// @param coord         the coordinate of the enclosed skeleton
  /// @param in            the input flow that should be passed to the
  ///                      enclosed skeleton upon continuation
  /// @param out           the output flow that should be passed to the
  ///                      enclosed skeleton upon continuation
  /// @param cur_stage     current stage of the paused skeleton. It is
  ///                      needed to determine the state upon resuming
  /// @param is_lazy       if this skeleton can stay untouched in the
  ///                      memento
  /// @param front         if this state should be inserted in the front
  ///                      of the memento double-ended queue
  //////////////////////////////////////////////////////////////////////
  template <typename Spawner, typename S,
            typename Coord,
            typename In, typename Out>
  void record_state(Spawner spawner,
                    S& skeleton,
                    std::size_t lid_offset,
                    Coord& skeleton_size, Coord& coord,
                    In const& in, Out const& out,
                    std::size_t cur_stage, bool is_lazy = false,
                    bool front = false)
  {
    element_type* callback =
      new skeletons_impl::memento_element<Spawner, S, Coord, In, Out>(
        spawner, skeleton, lid_offset, skeleton_size, coord, in, out,
        cur_stage);
    if (front)
      this->push_front(callback, is_lazy);
    else
      this->push_back(callback, is_lazy);

  }

  //////////////////////////////////////////////////////////////////////
  /// @brief This method resumes the spawning process of the front
  /// element of the memento dequeue as long as there is nothing else to
  /// do and the front element is not lazy. If all the elements of the
  /// memento double-ended queue are already resumed and there is nothing
  /// else left to do nothing will be done.
  ///
  /// The skeleton manager which holds this memento stack will finish
  /// the spawning process if there is nothing left to spawn in this
  /// memento stack.
  ///
  /// @param ignore_lazyness if true, continue the spawning process even
  ///                        if the front element of the memento is lazy
  //////////////////////////////////////////////////////////////////////
  void resume(bool ignore_lazyness = false);

private:
  void pop_keep();

  void push_back(element_type* element, bool is_lazy);

  void push_front(element_type* element, bool is_lazy);
};

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_EXECUTORS_MEMENTO_HPP
