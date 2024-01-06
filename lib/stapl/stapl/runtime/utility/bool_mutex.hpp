/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_BOOL_MUTEX_HPP
#define STAPL_RUNTIME_UTILITY_BOOL_MUTEX_HPP

#include "../exception.hpp"

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Bool based mutex.
///
/// This is an object that follows the concept of a @c std::mutex however it
/// does not guarantee thread safety. Example use case is protecting against
/// function re-entrancy.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
class bool_mutex
{
private:
  bool m_locked;

public:
  constexpr bool_mutex(void)
  : m_locked(false)
  { }

  bool_mutex(bool_mutex const&) = delete;
  bool_mutex& operator=(bool_mutex const&) = delete;

  void lock(void)
  {
    STAPL_RUNTIME_ASSERT(!m_locked);
    m_locked = true;
  }

  void unlock(void)
  {
    STAPL_RUNTIME_ASSERT(m_locked);
    m_locked = false;
  }

  bool try_lock(void) noexcept
  {
    if (m_locked)
      return false;
    m_locked = true;
    return true;
  }

  bool is_locked(void) const noexcept
  { return m_locked; }
};

} // namespace runtime

} // namespace stapl

#endif
