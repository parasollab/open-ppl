/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <cstdlib>

#include <stapl/runtime.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/numeric.hpp>

stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef stapl::array<unsigned long>          p_array_type;
  typedef stapl::array_view<p_array_type>      parrayView;

  int nelem = 1000;

  if (argc > 1)
    nelem = atoi(argv[1]);

  p_array_type parr(nelem);
  p_array_type parrRes(nelem);
  parrayView parrv(parr);
  parrayView parrResv(parrRes);

  stapl::generate(parrv, stapl::sequence<unsigned long>(1,0));

  // Default adjacent difference

  stapl::adjacent_difference(parrv,parrResv);

  int result = stapl::count(parrResv, size_t(0));
  stapl::do_once([=](void) {
    std::cout << argv[0];
    if (result == nelem-1)
      std::cout << " test1: PASSED\n";
    else
      std::cout << " test1: FAILED\n";
  });


  // adjacent difference with plus operator

  stapl::adjacent_difference(parrv, parrResv,
    stapl::plus<parrayView::value_type>());

  result = stapl::count(parrResv, size_t(2));

  stapl::do_once([=](void) {
    std::cout << argv[0];
    if (result == nelem-1)
      std::cout << " test2: PASSED\n";
    else
      std::cout << " test2: FAILED\n";
  });

  // adjacent difference with multilies operator

  stapl::adjacent_difference(parrv, parrResv,
    std::multiplies<parrayView::value_type>());

  result = stapl::count(parrResv, size_t(1));

  stapl::do_once([=](void) {
    std::cout << argv[0];
    if (result == nelem)
      std::cout << " test3: PASSED\n";
    else
      std::cout << " test3: FAILED\n";
  });

  return EXIT_SUCCESS;
}
