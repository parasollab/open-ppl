/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_TIMER_HPP
#define STAPL_RUNTIME_UTILITY_TIMER_HPP

#include <chrono>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Provides a timer mechanism.
///
/// The caller has to start the timer and check at regular intervals if it
/// expired. It is assumed that it has milliseconds accuracy but it is not
/// guaranteed that it will expire at the exact timeout.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<typename Clock = std::chrono::high_resolution_clock>
class timer
{
public:
  using duration   = std::chrono::milliseconds;
  using time_point = typename Clock::time_point;

private:
  duration   m_timeout;
  time_point m_timestamp;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the current time.
  //////////////////////////////////////////////////////////////////////
  static time_point now(void) noexcept
  { return Clock::now(); }

  explicit timer(const duration t = duration::zero()) noexcept
  : m_timeout(t)
  { }

  void reset(void) noexcept
  { m_timestamp = now(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Resets this @ref timer and sets a different timeout.
  //////////////////////////////////////////////////////////////////////
  void reset(const duration t) noexcept
  {
    m_timeout   = t;
    m_timestamp = now();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns if the @ref timer is expired.
  //////////////////////////////////////////////////////////////////////
  bool expired(const time_point t = now()) const noexcept
  {
    const auto diff = (t - m_timestamp);
    return (m_timeout <= diff);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the time until <tt>timer::expired()==true</tt> in
  ///        milliseconds.
  //////////////////////////////////////////////////////////////////////
  duration remaining(void) const noexcept
  {
    const auto t    = now();
    const auto diff = (t - m_timestamp);
    return std::chrono::duration_cast<duration>(m_timeout - diff);
   }
};

} // namespace runtime

} // namespace stapl

#endif
