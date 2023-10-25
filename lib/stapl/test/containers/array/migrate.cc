/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/array/array.hpp>

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  size_t sum = 0;
  size_t n = atoi(argv[1]);
  stapl::array<int> c(n);

  for (size_t i = 0; i < n; ++i)
    c.set_element(i, n-i);
  stapl::rmi_fence();

/*  for (size_t i = 0; i < n; ++i)
    sum += c.get_element(i);*/

  if (stapl::get_location_id() == 0)
    c.migrate(0, 1);
  else
    c.set_element(0, stapl::get_location_id()*1000);


  stapl::rmi_fence();

  std::cout << "sum: " << sum << std::endl;
  //std::cout << "elem 0 is on loc " << c.distribution().dir().find(0) << std::endl;

  stapl::rmi_fence();

  if (stapl::get_location_id() == 1) {
    stapl::future<int> e = c.get_element_split(0);
    std::cout << "elem 0 has value " << e.get() << std::endl;
  }

  /*if (stapl::get_location_id() == 1)
    c.migrate(0, 0);

  if (stapl::get_location_id() == 0) {
    stapl::pc_future<int> e = c.get_element_split(0);
    std::cout << "elem 0 has value " << e.get() << std::endl;
  }*/

  return EXIT_SUCCESS;
}
