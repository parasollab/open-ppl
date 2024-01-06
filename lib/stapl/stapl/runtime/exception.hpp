/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_EXCEPTION_HPP
#define STAPL_RUNTIME_EXCEPTION_HPP

#include <exception>
#include "config/debug.hpp"
#include <boost/current_function.hpp>

namespace stapl {

namespace runtime {

void assert_fail(const char* s,
                 const char* file, unsigned int line, const char* function);

void assert_fail(const char* s, const char* function);

void warning(const char* s, const char* function);


//////////////////////////////////////////////////////////////////////
/// @brief Returns the debug level of the runtime.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
int get_debug_level(void) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Runtime base exception object.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
class exception
: public std::exception
{
public:
  const char* what(void) const noexcept override
  { return "stapl::runtime::exception"; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Runtime error exception object.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
class runtime_error
: public exception
{
private:
  const char* m_what;

public:
  explicit runtime_error(const char* what_arg = "stapl::runtime::runtime_error")
  : m_what(what_arg)
  { }

  const char* what(void) const noexcept override
  { return m_what; }
};

} // namespace runtime

} // namespace stapl


//////////////////////////////////////////////////////////////////////
/// @brief Ensures the given input condition is true.
///
/// If @p E is @c true, nothing happens, otherwise the error message @p M is
/// printed and the program aborts.
///
/// This is not disabled by @c STAPL_NDEBUG or @c STAPL_RUNTIME_NDEBUG.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_CHECK(E, M)                                           \
  ( (E) ?                                                                   \
    (static_cast<void>(0)) :                                                \
    (stapl::runtime::assert_fail("STAPL Runtime Check Failed (" #E "): " M, \
                                 __FILE__, __LINE__, BOOST_CURRENT_FUNCTION)) )


//////////////////////////////////////////////////////////////////////
/// @brief Prints the given message and aborts execution.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_ERROR(M) \
  stapl::runtime::assert_fail("STAPL Runtime Error: " M, BOOST_CURRENT_FUNCTION)


//////////////////////////////////////////////////////////////////////
/// @brief Prints the given message as a warning.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
#define STAPL_RUNTIME_WARNING(M) \
  stapl::runtime::warning("STAPL Runtime Warning: " M, BOOST_CURRENT_FUNCTION)


#ifdef STAPL_RUNTIME_DEBUG


//////////////////////////////////////////////////////////////////////
/// @brief Ensures the given input condition is true.
///
/// If @p E is @c true, nothing happens, otherwise the @p E is printed and the
/// program aborts.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
# define STAPL_RUNTIME_ASSERT(E)                                            \
  ( (std::uncaught_exception() || (E)) ?                                    \
    (static_cast<void>(0)) :                                                \
    (stapl::runtime::assert_fail("STAPL Runtime Assertion (" #E ") failed", \
                                 __FILE__, __LINE__, BOOST_CURRENT_FUNCTION)) )


//////////////////////////////////////////////////////////////////////
/// @brief Ensures the given input condition is true.
///
/// If @p E is @c true, nothing happens, otherwise the error message @p M is
/// printed and the program aborts.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
# define STAPL_RUNTIME_ASSERT_MSG(E, M)                                \
  ( (std::uncaught_exception() || (E)) ?                               \
    (static_cast<void>(0)) :                                           \
    (stapl::runtime::assert_fail("STAPL Runtime Assertion: \"" M "\"", \
                                 __FILE__, __LINE__, BOOST_CURRENT_FUNCTION)) )


#else


//////////////////////////////////////////////////////////////////////
/// @brief Ensures the given input condition is true.
///
/// If @p E is @c true, nothing happens, otherwise the @p E is printed and the
/// program aborts.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
# define STAPL_RUNTIME_ASSERT(E) \
  static_cast<void>(0)


//////////////////////////////////////////////////////////////////////
/// @brief Ensures the given input condition is true.
///
/// If @p E is @c true, nothing happens, otherwise the error message @p M is
/// printed and the program aborts.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
# define STAPL_RUNTIME_ASSERT_MSG(E, M) \
  static_cast<void>(0)


#endif // STAPL_RUNTIME_DEBUG

#endif
