/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COUNTER_PAPI_PAPI_TIMER_N_COUNTER_HPP
#define STAPL_RUNTIME_COUNTER_PAPI_PAPI_TIMER_N_COUNTER_HPP

#include "papi_timer.hpp"
#include "papi_counter.hpp"
#include "../../serialization.hpp"
#include <initializer_list>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Counter that uses PAPI to measure both time and different events.
///
/// This counter provides support for measuring various events using PAPI. The
/// number and type of events is only limited by the hardware support.
///
/// The events are identified by their PAPI name passed at the constructor of
/// the counter.
///
/// PAPI event names can be found using the @c papi_avail utility.
///
/// @see papi_counter, papi_timer
///
/// @ingroup counters
//////////////////////////////////////////////////////////////////////
class papi_timer_n_counter
{
public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Counter return type that stores time and requested counters.
  //////////////////////////////////////////////////////////////////////
  struct raw_value_type
  {
    typedef double                   time_type;
    typedef papi_counter::value_type counters_type;

    time_type     time;
    counters_type counters;

    raw_value_type(void)
    : time(0.0)
    { }

    raw_value_type(time_type const& t, counters_type const& c)
    : time(t),
      counters(c)
    { }

    raw_value_type(time_type const& t, counters_type& c)
    : time(t),
      counters(std::move(c))
    { }

    void define_type(typer& t)
    {
      t.member(time);
      t.member(counters);
    }
  };

  typedef raw_value_type value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the name of the counter as a C string.
  //////////////////////////////////////////////////////////////////////
  static constexpr const char* name(void) noexcept
  { return "PAPI timer_n_counter"; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Normalizes the given raw value.
  //////////////////////////////////////////////////////////////////////
  static value_type normalize(raw_value_type v) noexcept
  { return v; }

private:
  papi_timer   m_timer;
  papi_counter m_counter;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref papi_timer_n_counter with the requested PAPI
  ///        events.
  //////////////////////////////////////////////////////////////////////
  template<typename... Args>
  explicit papi_timer_n_counter(Args&&... events)
  : m_counter(std::forward<Args>(events)...)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref papi_timer_n_counter with the requested PAPI
  ///        events.
  //////////////////////////////////////////////////////////////////////
  explicit papi_timer_n_counter(std::initializer_list<int> events)
  : m_counter(events)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the raw value from the counter.
  //////////////////////////////////////////////////////////////////////
  raw_value_type read(void) const
  { return raw_value_type(m_timer.read(), m_counter.read()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Starts the counter.
  //////////////////////////////////////////////////////////////////////
  void start(void)
  {
    m_counter.start();
    m_timer.start();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Stops the counter and returns the difference from @ref start().
  //////////////////////////////////////////////////////////////////////
  value_type stop(void)
  { return normalize(raw_value_type(m_timer.stop(), m_counter.stop())); }
};


inline papi_timer_n_counter::value_type&
operator+=(papi_timer_n_counter::value_type& x,
           papi_timer_n_counter::value_type const& y) noexcept
{
  x.time     += y.time;
  x.counters += y.counters;
  return x;
}

} // namespace stapl

#endif
