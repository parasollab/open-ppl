/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_EXIT_CODE_HPP
#define STAPL_RUNTIME_EXIT_CODE_HPP

#include <cstdlib>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Class to return exit codes from STAPL applications.
///
/// A @c 0 exit code requires SPMD creation of an @ref exit_code object, whereas
/// a non @c 0 exit code does not. As such, the following uses are correct:
/// @code
/// stapl::exit_code stapl_main(int, char*[])
/// {
///   return EXIT_SUCCESS;
/// }
///
/// stapl::exit_code stapl_main(int, char*[])
/// {
///   return EXIT_FAILURE;
/// }
///
/// stapl::exit_code stapl_main(int, char*[])
/// {
///   if (condition)
///     return EXIT_FAILURE;
///   return EXIT_SUCCESS;
/// }
/// @endcode
/// whereas the following is not:
/// @code
/// stapl::exit_code stapl_main(int, char*[])
/// {
///   if (condition)
///     return EXIT_SUCCESS;
///   return EXIT_FAILURE;
/// }
/// @endcode
///
/// @warning Users should not instantiate objects of this class explicitly.
///          Instances should only be instantiated through implicit conversion
///          from @c int.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
class exit_code
{
private:
  int m_code;

public:
  exit_code(int code);

  exit_code& operator=(exit_code const&) = delete;

  //////////////////////////////////////////////////////////////////////
  /// @brief Waits for all locations to create an instance of @ref exit_code and
  ///        call @ref wait().
  ///
  /// @warning This function has to be called outside the scope the initial
  ///          @ref exit_code object was created in.
  //////////////////////////////////////////////////////////////////////
  void wait(void) const;

  int get_code(void) const noexcept
  { return m_code; }

  bool operator==(const int code) const noexcept
  { return (m_code==code); }

  bool operator!=(const int code) const noexcept
  { return !(*this==code); }
};

} // namespace stapl

#endif
