/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/set/set.hpp>
#include "../../test_report.hpp"

using namespace stapl;

//SET_INSERT_FN Uncomment if a use case is found.
#if 0
struct test_wf
{
  typedef void result_type;

  template<typename T1, typename T2>
  void operator()(T1& t, T2 const& nval) const
  {}
};
#endif

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  const size_t n = atoi(argv[1]);

  typedef stapl::set<char> container_type;
  typedef indexed_domain<char>             domain_type;

  domain_type dom('a', 'z');
  container_type m(dom);

  for (size_t i = 0; i < n; ++i)
  {
  //SET_INSERT_FN Uncomment if a use case is found.
  #if 0
    m.insert('a', test_wf());
  #else
    m.insert('a');
  #endif
  }

  rmi_fence();

  STAPL_TEST_REPORT(m.size() == 1, "Test one value inserted")

  //SET_INSERT_FN Uncomment if a use case is found.
  #if 0
  STAPL_TEST_REPORT(true, "Test insert with functor")
  #else
  STAPL_TEST_REPORT(true, "Test insert")
  #endif

  return EXIT_SUCCESS;
}
