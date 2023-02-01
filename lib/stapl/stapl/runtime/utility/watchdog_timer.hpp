/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_UTILITY_WATCHDOG_TIMER_HPP
#define STAPL_RUNTIME_UTILITY_WATCHDOG_TIMER_HPP

#include <atomic>
#include <functional>
#include <thread>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Allows the creation of a watchdog timer that invokes the given
///        function every @p interval milliseconds.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
class watchdog_timer
{
private:
  typedef std::function<void(void)> function_type;

  function_type     m_f;
  std::atomic<bool> m_active;
  std::thread       m_thread;

public:
  watchdog_timer(void)
  : m_active(false)
  { }

  template<typename Function, typename Interval>
  watchdog_timer(Function&& f, Interval&& interval)
  : m_active(false)
  { start(std::forward<Function>(f), std::forward<Interval>(interval)); }

  template<typename Function, typename Interval>
  void start(Function&& f, Interval&& interval)
  {
    if (this->m_active.load(std::memory_order_relaxed))
      stop();
    m_f      = std::forward<Function>(f);
    m_active = true;
    m_thread = std::thread([this, interval]
               {
                 do {
                   std::this_thread::sleep_for(interval);
                   this->m_f();
                 } while (this->m_active.load(std::memory_order_relaxed));
               });
  }

  void stop(void)
  {
    m_active.store(false, std::memory_order_relaxed);
    m_thread.join();
  }

  ~watchdog_timer(void)
  { stop(); }
};

} // namespace stapl

#endif
