/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Benchmark for @ref stapl::construct().
///
/// The following are benchmarked:
/// -# @ref stapl::construct() on all locations.
/// -# @ref stapl::construct() on half the locations.
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#include <boost/lexical_cast.hpp>

class A
: public stapl::p_object
{ };

struct construct_all_wf
{
  typedef void result_type;

  static std::string name(void)
  {
    return std::string("construct(all_locations) " +
           boost::lexical_cast<std::string>(stapl::get_num_locations()));
  }

  void operator()(void)
  {
    stapl::future<stapl::rmi_handle::reference> f =
      stapl::construct<A>(stapl::neighbor_locations);
    stapl::rmi_handle::reference r = f.get();
    stapl::p_object_delete<A> d;
    d(r);
    stapl::rmi_fence(); // wait for p_object to be deleted
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

  {
    construct_all_wf wf;
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}
