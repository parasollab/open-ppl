/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SERIAL_HPP
#define STAPL_SKELETONS_SERIAL_HPP

#include <stapl/runtime/executor/scheduler/default_info.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/serial.hpp>
#include <stapl/skeletons/transformations/coarse/serial.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>
#include <stapl/views/metadata/coarseners/default.hpp>
#include <stapl/views/metadata/coarseners/multiview.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Trivial task placement policy used by @ref serial_io that
/// forces all tasks to run on location 0.
///
/// As written now, all tasks placed on location 0 of gang executing
/// stapl_main.
///
/// @todo Replace explicit affinity tag creation with information from
///       system view of affinities when available.
///
/// @ingroup skeletonsSerial
//////////////////////////////////////////////////////////////////////
struct location_0_placement_policy
{
  template<typename WF, typename ...Args>
  locality_info execution_location(WF const&, Args const&...) const
  {
    return locality_info(
      LQ_CERTAIN, affinity_tag::make_tag(0),
      get_anonymous_executor().get_rmi_handle(), location_type(0)
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return priority of the PARAGRAPH being scheduled.
  ///
  /// Method required by paragraph::operator() when the executor for the
  /// paragraph is added to the parent executor.
  //////////////////////////////////////////////////////////////////////
  constexpr default_info get_sched_info(void) const noexcept
  { return default_info(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Hook to allow tracking of when tasks are executed.
  //////////////////////////////////////////////////////////////////////
  void notify_finished(void) noexcept
  { }
}; // struct location_0_placement_policy

} // namespace detail

namespace serial_impl {


//////////////////////////////////////////////////////////////////////
/// @brief Construct and execute a serial skeleton that performs a serial
/// computation on the views provided.
///
/// This specialization is used when only the input views need to be
/// coarsened.
///
/// @param wf       operation to apply to each set of view elements.
/// @param num_sets the number of sets to form. View elements are balance
///                 distributed across the sets.
/// @param v        views to be processed.
///
/// @ingroup skeletons
//////////////////////////////////////////////////////////////////////
template <typename Scheduler = default_scheduler,
          typename WF, typename... V>
void serial(skeletons::tags::with_coarsened_wf,
            std::size_t num_sets, WF&& wf, V&&... v)
{
  skeletons::execute(
    skeletons::execution_params(default_coarsener(), stapl::use_default(),
                                Scheduler()),
    skeletons::serial<sizeof...(V)>(std::forward<WF>(wf), num_sets),
    std::forward<V>(v)...);
}

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief skeletons_impl::serial.
///
/// This specialization is used when neither the skeleton nor the view
/// need to be coarsened.
///
/// @param wf       operation to apply to each set of view elements.
/// @param num_sets the number of sets to form. View elements are balance
///                 distributed across the sets.
/// @param v        views to be processed.
///
/// @ingroup skeletons
//////////////////////////////////////////////////////////////////////
template <typename Scheduler = default_scheduler,
          typename WF, typename... V>
void serial(skeletons::tags::no_coarsening, std::size_t num_sets,
            WF&& wf, V&&... v)
{
  skeletons::execute(
    skeletons::execution_params(null_coarsener(), stapl::use_default(),
                                Scheduler()),
    skeletons::serial<sizeof...(V)>(std::forward<WF>(wf), num_sets),
    std::forward<V>(v)...);
}

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief skeletons_impl::serial.
///
/// This specialization is used when both the skeleton and the view
/// have to be coarsened.
///
/// @param wf       operation to apply to each set of view elements.
/// @param num_sets the number of sets to form. View elements are balance
///                 distributed across the sets.
/// @param v        views to be processed.
///
/// @ingroup skeletons
//////////////////////////////////////////////////////////////////////
template <typename Scheduler = default_scheduler,
          typename WF, typename... V>
void serial(stapl::use_default, std::size_t num_sets, WF&& wf,
            V&&... v)
{
  skeletons::execute(
    skeletons::execution_params(default_coarsener(), stapl::use_default(),
                                 Scheduler()),
    skeletons::coarse(
      skeletons::serial<sizeof...(V)>(
        std::forward<WF>(wf), num_sets)),
    std::forward<V>(v)...);
}

} // serial_impl


//////////////////////////////////////////////////////////////////////
/// @brief Construct and execute a skeleton that performs a set of serial
/// computations on the views provided. The tasks of the serial computation
/// are divided into the @p nsets specified. Tasks within a set are processed
/// serially, while each set is processed concurrently.
///
/// An example use case is writing n view elements to m files.  Each set of
/// elements [0, m-1], [m, 2m-1], ... will be executed concurrently, and
/// elements within a set will be processed serially to ensure that the write
/// of one element to the file completes before the next element to be written
/// to the file begins writing.
///
/// @param wf The work function to be applied on each elements of sets
/// @param nsets The number of sets to form. View elements are balance
/// distributed across the sets.
/// @param v  Views to be processed.
///
/// @ingroup skeletons
//////////////////////////////////////////////////////////////////////
template <typename Tag = stapl::use_default,
          typename WF, typename... V>
void
serial_sets(WF&& wf, int nsets, V&&... v)
{
  serial_impl::serial(Tag(), nsets, std::forward<WF>(wf),
                      std::forward<V>(v)...);
}

//////////////////////////////////////////////////////////////////////
/// @brief Specialized form of @ref serial_sets that schedules all tasks
/// created by the skeleton on location chosen by the task placement
/// policy in effect
///
/// @param wf    The work function to be applied on each element of the views
///              provided.
/// @param v     Views to be processed.
///
/// @ingroup skeletons
//////////////////////////////////////////////////////////////////////
template <typename Tag = stapl::use_default,
          typename WF, typename... V>
void
serial(WF&& wf, V&&... v)
{
  serial_impl::serial(Tag(), 0, std::forward<WF>(wf), std::forward<V>(v)...);
}

//////////////////////////////////////////////////////////////////////
/// @brief Specialized form of @ref serial_sets that schedules all tasks
/// created by the skeleton on location 0 for execution using
/// @ref location_0_placement_policy task placement policy.
///
/// @param wf The work function to be applied on each element of the views
///           provided.
/// @param v  Views to be processed.
///
/// @ingroup skeletons
//////////////////////////////////////////////////////////////////////
template <typename Tag = stapl::use_default,
          typename WF, typename... V>
void serial_io(WF&& wf, V&&... v)
{
  serial_impl::serial<
    fifo_scheduler<detail::location_0_placement_policy>
  >(Tag(), 0, std::forward<WF>(wf), std::forward<V>(v)...);
}

} // namespace stapl

#endif  // STAPL_SKELETONS_SERIAL_HPP
