/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_ASSERT_HPP
#define STAPL_RUNTIME_ASSERT_HPP

#if !defined(STAPL_NDEBUG)


# ifdef _STAPL

#  include <exception>
#  include <boost/current_function.hpp>

namespace stapl {

namespace runtime {

void assert_fail(const char* s,
                 const char* file, unsigned int line, const char* function);

void warning(const char* s,
             const char* file, unsigned int line, const char* function);

} // namespace runtime

} // namespace stapl


//////////////////////////////////////////////////////////////////////
/// @brief Ensures the given input condition is true.
///
/// If @p E is @c true, nothing happens, otherwise the error message @p M is
/// printed and the program aborts.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
#  define stapl_assert(E, M)                            \
  ( (std::uncaught_exception() || (E)) ?                \
    (static_cast<void>(0)) :                            \
    (stapl::runtime::assert_fail("STAPL ASSERTION: " M, \
                                 __FILE__, __LINE__, BOOST_CURRENT_FUNCTION)) )

//////////////////////////////////////////////////////////////////////
/// @brief Ensures the given input condition is true.
///
/// If @p E is @c true, nothing happens, otherwise the warning message @p M is
/// printed.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
#  define stapl_warning(E, M)                     \
  ( (std::uncaught_exception() || (E)) ?          \
    (static_cast<void>(0)) :                      \
    (stapl::runtime::warning("STAPL WARNING: " M, \
                             __FILE__, __LINE__, BOOST_CURRENT_FUNCTION)) )

# else // _STAPL

#  include <cstdio>
#  include <cstdlib>
#  include <exception>

//////////////////////////////////////////////////////////////////////
/// @brief Ensures the given input condition is true.
///
/// If @p E is @c true, nothing happens, otherwise the error message @p M is
/// printed and the program aborts.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
#  define stapl_assert(E, M) do {                                        \
    if ( !std::uncaught_exception() && !(E) ) {                          \
      std::fprintf(stderr, "STAPL ASSERTION: %s (file: %s, line: %d)\n", \
                   M, __FILE__, __LINE__);                               \
      std::abort();                                                      \
    }                                                                    \
  } while (false)

//////////////////////////////////////////////////////////////////////
/// @brief Ensures the given input condition is true.
///
/// If @p E is @c true, nothing happens, otherwise the warning message @p M is
/// printed.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
#  define stapl_warning(E, M)                                         \
  ( (std::uncaught_exception() || (E)) ?                              \
    (static_cast<void>(0)) :                                          \
    (std::fprintf(stderr, "STAPL WARNING: %s (file: %s, line: %d)\n", \
                  M, __FILE__, __LINE__)) )

#  endif // _STAPL


#else // STAPL_NDEBUG


//////////////////////////////////////////////////////////////////////
/// @brief Ensures the given input condition is true.
///
/// If @p E is true, nothing happens, otherwise the error message @p M is
/// printed and the program aborts.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
# define stapl_assert(E, M) static_cast<void>(0)

//////////////////////////////////////////////////////////////////////
/// @brief Ensures the given input condition is true.
///
/// If @p E is @c true, nothing happens, otherwise the warning message @p M is
/// printed.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
# define stapl_warning(E, M) static_cast<void>(0)


#endif // STAPL_NDEBUG

#endif
