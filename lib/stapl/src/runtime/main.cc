/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/runtime.hpp>
#include <stapl/runtime/executor/anonymous_executor.hpp>
#include <cstdlib>
#include <limits>

namespace stapl {

void main_wf::operator()(void)
{
  anonymous_executor e; // FIXME remove this
  const exit_code ex = m_f(m_argc, m_argv);
  if (ex!=EXIT_SUCCESS) {
    std::exit(ex.get_code()); // FIXME
  }
  ex.wait();
}

} // namespace stapl


#ifndef NO_STAPL_MAIN

//////////////////////////////////////////////////////////////////////
/// @brief The starting point for SPMD user code execution.
///
/// It replaces the sequential equivalent:
/// @code int main(int argc, char* argv[]) @endcode
///
/// @ingroup ARMI
//////////////////////////////////////////////////////////////////////
extern stapl::exit_code stapl_main(int argc, char *argv[]);


int main(int argc, char *argv[])
{
  using namespace stapl;

  option opt{argc, argv};
  initialize(opt);
  execute(main_wf{argc, argv, &stapl_main},
          opt.get<unsigned int>("STAPL_MAIN_LEVELS",
                                std::numeric_limits<unsigned int>::max()));
  finalize();

  return EXIT_SUCCESS;
}

#endif
