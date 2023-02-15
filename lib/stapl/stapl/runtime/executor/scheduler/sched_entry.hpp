/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_SCHEDULER_SCHED_ENTRY_HPP
#define STAPL_RUNTIME_EXECUTOR_SCHEDULER_SCHED_ENTRY_HPP

#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Base scheduler entry class for entries that can know their scheduling
///        information.
///
/// @tparam SchedInfo Scheduling information object type.
/// @tparam Hook      Hook for storing the entry in an intrusive container.
///
/// For example, tasks created by a @ref paragraph can have scheduling
/// information, whereas @ref gang_executor objects cannot.
///
/// @see executor
/// @ingroup scheduling
//////////////////////////////////////////////////////////////////////
template<typename SchedInfo, typename Hook>
class sched_entry
  : public Hook,
    private SchedInfo
{
public:
  typedef SchedInfo sched_info_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref sched_entry.
  ///
  /// @param sched_info Scheduling information.
  //////////////////////////////////////////////////////////////////////
  explicit sched_entry(sched_info_type sched_info)
    : SchedInfo(std::move(sched_info))
  { }

  sched_entry(sched_entry const&) = delete;
  sched_entry& operator=(sched_entry const&) = delete;

protected:
  ~sched_entry(void) = default;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the scheduling information associated with this entry.
  //////////////////////////////////////////////////////////////////////
  sched_info_type const& sched_info(void) const noexcept
  { return *this; }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc sched_info(void) const
  //////////////////////////////////////////////////////////////////////
  sched_info_type& sched_info(void) noexcept
  { return *this; }
};

} // namespace stapl

#endif
