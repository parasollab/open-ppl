/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COUNTER_PAPI_PAPI_COUNTER_HPP
#define STAPL_RUNTIME_COUNTER_PAPI_PAPI_COUNTER_HPP

#include "../config.hpp"
#include "../../exception.hpp"
#include <papi.h>
#include <vector>
#include <initializer_list>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Counter that uses PAPI to measure different events.
///
/// This counter provides support for measuring various events using PAPI. The
/// number and type of events is only limited by the hardware support.
///
/// The events are identified by their PAPI name passed at the constructor of
/// the counter.
///
/// PAPI event names can be found using the @c papi_avail utility.
///
/// @ingroup counters
//////////////////////////////////////////////////////////////////////
class papi_counter
{
public:
  typedef std::vector<long_long> raw_value_type;
  typedef std::vector<long_long> value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the name of the counter as a C string.
  //////////////////////////////////////////////////////////////////////
  static constexpr const char* name(void) noexcept
  { return "PAPI counter"; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Normalizes the given raw value.
  //////////////////////////////////////////////////////////////////////
  static constexpr value_type const& normalize(raw_value_type const& v) noexcept
  { return v; }

private:
  int        m_events;
  value_type m_vals;

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the given events to the vector of events.
  //////////////////////////////////////////////////////////////////////
  void add_events(std::initializer_list<int> events)
  {
    STAPL_RUNTIME_CHECK(PAPI_create_eventset(&m_events)==PAPI_OK,
                        "PAPI_create_eventset() failed.");
    m_vals.resize(events.size());
    for (auto e : events) {
      STAPL_RUNTIME_CHECK(PAPI_add_event(m_events, e)==PAPI_OK,
                          "PAPI_add_event() failed.");
    }
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref papi_counter with the requested PAPI events.
  //////////////////////////////////////////////////////////////////////
  template<typename... Args>
  explicit papi_counter(Args&&... events)
  : m_events(PAPI_NULL)
  { add_events({ std::forward<Args>(events)... }); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref papi_counter with the requested PAPI events in
  ///        the @c std::initializer_list.
  //////////////////////////////////////////////////////////////////////
  explicit papi_counter(std::initializer_list<int> l)
  : m_events(PAPI_NULL)
  { add_events(l); }

  ~papi_counter(void)
  {
    STAPL_RUNTIME_CHECK(PAPI_cleanup_eventset(m_events)==PAPI_OK,
                        "PAPI_cleanup_eventset() failed.");
    STAPL_RUNTIME_CHECK(PAPI_destroy_eventset(&m_events)==PAPI_OK,
                        "PAPI_destroy_eventset() failed.");
  }

  papi_counter(papi_counter const&) = delete;
  papi_counter& operator=(papi_counter const&) = delete;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the raw value from the counter.
  //////////////////////////////////////////////////////////////////////
  raw_value_type read(void) const
  {
    std::vector<long_long> events(m_vals.size());
    STAPL_RUNTIME_CHECK(PAPI_read(m_events, &events[0])==PAPI_OK,
                        "Could not read counters");
    return events;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Starts the counter.
  //////////////////////////////////////////////////////////////////////
  void start(void) noexcept
  {
    STAPL_RUNTIME_CHECK(PAPI_start(m_events)==PAPI_OK,
                        "Could not start counters");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Stops the counter and returns the difference from @ref start().
  ///
  /// @return A @c std::vector<long_long> with the difference from @ref start().
  //////////////////////////////////////////////////////////////////////
  value_type stop(void)
  {
    STAPL_RUNTIME_CHECK(PAPI_stop(m_events, &m_vals[0])==PAPI_OK,
                        "Could not stop counters");
    return normalize(m_vals);
  }
};


inline papi_counter::value_type&
operator+=(papi_counter::value_type& x,
           papi_counter::value_type const& y) noexcept
{
  if (x.size()==0) {
    x = y;
  }
  else {
    STAPL_RUNTIME_CHECK(x.size()==y.size(), "Sizes are not the same");
    for (papi_counter::value_type::size_type i=0; i<y.size(); ++i)
      x[i] += y[i];
  }
  return x;
}

} // namespace stapl

#endif
