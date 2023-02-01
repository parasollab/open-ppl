/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>

#include <stapl/skeletons/map.hpp>
#include <stapl/runtime.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/utility/do_once.hpp>


struct work_func
{
  typedef void result_type;

  template <typename Reference>
  void operator()(Reference ref) const
  {
    ref = ref + 1;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef stapl::array<int>          array_t;
  typedef stapl::array_view<array_t> view_t;

  const int n_locs = stapl::get_num_locations();

  stapl::do_once([=](void) {
    std::cout << "Testing simple executor call with "
              << n_locs << " locations... ";
  });

  array_t   arr(100);
  view_t    view(arr);

  stapl::map_func(work_func(), view);

  stapl::do_once([](void) {
    std::cout << "Passed\n";
  });

  return EXIT_SUCCESS;
}
