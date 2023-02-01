/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_MAIN_HPP
#define STAPL_RUNTIME_MAIN_HPP

#include "exit_code.hpp"
#include <functional>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Function object that is used with @ref execute() to initiate a STAPL
///        application.
///
/// @ingroup ARMIUtilities
///
/// @todo It is not yet clear if it should be replaced by a @ref paragraph or
///       not.
//////////////////////////////////////////////////////////////////////
class main_wf
{
private:
  std::function<exit_code(int, char*[])> m_f;
  int                                    m_argc;
  char**                                 m_argv;

public:
  template<typename Function>
  main_wf(int argc, char* argv[], Function&& f)
  : m_f(std::forward<Function>(f)),
    m_argc(argc),
    m_argv(argv)
  { }

  void operator()(void);
};

} // namespace stapl

#endif
