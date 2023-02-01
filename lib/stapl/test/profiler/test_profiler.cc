/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/profiler/base_profiler.hpp>

using namespace stapl;

typedef counter<default_timer> counter_type;

class simple_profiler
  : public base_profiler<counter_type>
{
private:
  typedef base_profiler<counter_type> base_type;

public:
  simple_profiler(std::string s, int argc, char** argv)
    : base_type(s, "STAPL", argc, argv)
  { }

  void initialize_iteration(void)
  {
    // optional
    // code to be run before executing each iteration
  }

  void run(void)
  {
    // mandatory
    // code to be profiled
  }

  void finalize_iteration(void)
  {
    // optional
    // code to be run after the run method finishes;
    // for example clean up or checks for correctness
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  simple_profiler oprep("simple_test", argc, argv);
  oprep.collect_profile(); // run the experiment and collect information
  oprep.report();          // default reporting goes into P files on the disk
                           // with the prefix profile_simple_test;
  return EXIT_SUCCESS;
}
