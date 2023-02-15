/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_COUNTER_COUNTER_HPP
#define STAPL_RUNTIME_COUNTER_COUNTER_HPP

#include "config.hpp"
#include "../exception.hpp"
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Controls enabling/disabling a specific group of @ref counter objects
///        at runtime.
///
/// @tparam GroupID The @ref counter object group id.
///
/// @see counter
/// @ingroup counters
//////////////////////////////////////////////////////////////////////
template<int GroupID>
class counter_group
{
protected:
  static bool s_active;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Enables the counter group.
  //////////////////////////////////////////////////////////////////////
  static void enable(void) noexcept
  { s_active = true; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Disables the counter group.
  //////////////////////////////////////////////////////////////////////
  static void disable(void) noexcept
  { s_active = false; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the counter group is enabled.
  //////////////////////////////////////////////////////////////////////
  static bool enabled(void) noexcept
  { return s_active; }
};
// by default, all counter groups are enabled
template<int GroupID> bool counter_group<GroupID>::s_active = true;


//////////////////////////////////////////////////////////////////////
/// @brief Disables a specific counter group at compile-time.
///
/// If counters that are in the same group need to be disabled at compile time,
/// resulting in empty calls without overhead, then a specialization of the
/// class @ref disable_group_counter must be provided that extends from
/// @c std::true_type.
///
/// For example, if counters with group id @c 0 must be disabled, then defining
/// @code
/// namespace stapl {
///
/// template<>
/// struct disable_group_counter<0>
/// : public std::true_type
/// { };
///
/// }
/// @endcode
/// will disable them.
///
/// @ingroup counters
//////////////////////////////////////////////////////////////////////
template<int GroupID>
struct disable_group_counter
: public std::false_type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Implements a configurable counter.
///
/// @tparam T       The counter type.
/// @tparam GroupID The counter group the counter belongs to.
///
/// The @ref counter objects have the ability to start, stop and resume
/// collecting information. They have to be explicitly reset through the
/// @ref counter::reset() function to clear their internal state.
///
/// @ingroup counters
//////////////////////////////////////////////////////////////////////
template<typename T,
         int GroupID = 0,
         bool = disable_group_counter<GroupID>::value>
class counter
{
public:
  typedef T                                 counter_type;
  typedef typename counter_type::value_type value_type;
  typedef unsigned int                      size_type;
  typedef int                               group_id_type;

  /// The group id the counter belongs to.
  static constexpr group_id_type group_id = GroupID;

protected:
  typedef counter_group<GroupID> group_type;

  size_type    m_calls;
  value_type   m_value;
  counter_type m_counter;
  bool         m_active;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the native name of the counter as a C string.
  //////////////////////////////////////////////////////////////////////
  static constexpr const char* native_name(void) noexcept
  { return counter_type::name(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref counter and forwards the arguments to the
  ///        underlying counter.
  //////////////////////////////////////////////////////////////////////
  template<typename... Args>
  counter(Args&&... args)
  : m_calls(0),
    m_value(),
    m_counter(std::forward<Args>(args)...),
    m_active(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reading of this counter.
  //////////////////////////////////////////////////////////////////////
  value_type read(void)
  { return counter_type::normalize(m_counter.read()); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the cumulative difference between calls of @ref start() and
  ///        @ref stop() since the last call to @ref reset().
  //////////////////////////////////////////////////////////////////////
  value_type value(void) const noexcept
  { return m_value; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of times this counter has been started and
  ///        stopped since the last call to @ref reset().
  //////////////////////////////////////////////////////////////////////
  constexpr size_type calls(void) const noexcept
  { return m_calls; }

  constexpr bool active(void) const noexcept
  { return m_active; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Starts this counter.
  ///
  /// @warning @ref start() can be called only after construction or a call to
  ///          @ref stop().
  //////////////////////////////////////////////////////////////////////
  void start(void)
  {
    if (group_type::enabled()) {
      STAPL_RUNTIME_CHECK(!m_active, "Counter not idle");
      m_active = true;
      ++m_calls;
      m_counter.start();
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Restarts this counter.
  ///
  /// This function is equivalent to calling @ref reset() and @ref start().
  ///
  /// @warning @ref restart() can be called only after construction or a call to
  ///          @ref stop().
  //////////////////////////////////////////////////////////////////////
  void restart(void)
  {
    reset();
    start();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Stops this counter.
  ///
  /// @warning @ref stop can be called only after a call to @ref start().
  ///
  /// @return The cumulative difference between calls of @ref start() and
  ///         @ref stop() since the last call to @ref reset().
  //////////////////////////////////////////////////////////////////////
  value_type stop(void)
  {
    if (group_type::enabled()) {
      m_value += m_counter.stop();
      STAPL_RUNTIME_CHECK(m_active, "Counter idle");
      m_active = false;
    }
    else {
      STAPL_RUNTIME_CHECK(!m_active, "Counter disabled while active");
    }
    return m_value;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Resets this counter.
  //////////////////////////////////////////////////////////////////////
  void reset(void)
  {
    m_calls = 0;
    m_value = value_type{};
  }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref counter when its group is disabled at
///        compile-time through @ref disable_group_counter.
///
/// @ingroup counters
//////////////////////////////////////////////////////////////////////
template<typename T, int GroupID>
class counter<T, GroupID, true>
{
public:
  typedef T                                 counter_type;
  typedef typename counter_type::value_type value_type;
  typedef unsigned int                      size_type;
  typedef int                               group_id_type;

  static constexpr group_id_type group_id = GroupID;

  static constexpr const char* native_name(void) noexcept
  { return counter_type::name(); }

  template<typename... Args>
  counter(Args&&...)
  { }

  value_type read(void)
  { return value_type{}; }

  value_type value(void) const noexcept
  { return value_type{}; }

  constexpr size_type calls(void) const noexcept
  { return 0; }

  void start(void)
  { }

  void restart(void)
  { }

  value_type stop(void)
  { return value_type(); }

  void reset(void)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object to start and stop a @ref counter object in a specific
///        scope.
///
/// @ingroup counters
//////////////////////////////////////////////////////////////////////
template<typename Counter>
class scoped_counter
{
private:
  Counter& m_counter;

public:
  explicit scoped_counter(Counter& c, const bool cumulative = true)
  : m_counter(c)
  {
    if (cumulative)
      m_counter.start();
    else
      m_counter.restart();
  }

  ~scoped_counter(void)
  { m_counter.stop(); }
};

} // namespace stapl

#endif
