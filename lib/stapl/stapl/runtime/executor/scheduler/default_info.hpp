/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_SCHEDULER_DEFAULT_INFO_HPP
#define STAPL_RUNTIME_EXECUTOR_SCHEDULER_DEFAULT_INFO_HPP

#include "../../serialization_fwd.hpp"
#include "../../tags.hpp"
#include "../../stapl_assert.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default scheduling information.
///
/// Priority-based scheduling information based on <tt>unsigned int</tt>.
///
/// @ingroup scheduling
//////////////////////////////////////////////////////////////////////
class default_info
{
public:
  using priority_type = unsigned int;

private:
  priority_type m_priority;

public:
  constexpr default_info(none_t) noexcept
    : m_priority(0)
  { }

  constexpr explicit default_info(priority_type priority = 0) noexcept
    : m_priority(priority)
  { }

  operator none_t(void) const noexcept
  {
    stapl_assert(m_priority == 0, "converting non-zero priority to none_t");
    return none;
  }

  constexpr priority_type priority(void) const noexcept
  { return m_priority; }

  void define_type(typer& t)
  { t.member(m_priority); }

  friend constexpr bool operator==(default_info const& x,
                                   default_info const& y) noexcept
  { return (x.m_priority == y.m_priority); }

  friend constexpr bool operator<(default_info const& x,
                                  default_info const& y) noexcept
  { return (x.m_priority < y.m_priority); }
};

} // namespace stapl

#endif
