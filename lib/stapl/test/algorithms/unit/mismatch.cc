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
#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/generators/functor.hpp>
#include <stapl/utility/do_once.hpp>

#include "../../test_report.hpp"

using namespace stapl;

struct rand_func
{
  typedef size_t    index_type;
  typedef size_t    result_type;

  rand_func()
  {
    srand(get_location_id());
  }

  result_type operator()(index_type) const
  {
    return rand();
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef array<size_t>                   array_type;
  typedef array_view<array_type>          array_view_t;
  typedef functor_container<rand_func>    rand_container_t;
  typedef array_view<rand_container_t>    rand_view;

  unsigned int nelem = 1000;
  if (argc>1) nelem = atoi(argv[1]);
  unsigned int mism = nelem/2;

  rand_container_t rands(nelem,rand_func());
  rand_view randv(rands);

  array_type array1(nelem);
  array_type array2(nelem);
  array_view_t view1(array1);
  array_view_t view2(array2);

  // Create arrays & confirm no mismatch
  copy(randv, view1);
  copy(view1, view2);

  std::pair<array_view_t::reference, array_view_t::reference> result1 =
    mismatch(view1, view2);

  bool good[4];
  good[0] = is_null_reference(result1.first);
  good[1] = is_null_reference(result1.second);

  // Mismatch second array & confirm
  --view2[mism];
  std::pair<array_view_t::reference, array_view_t::reference> result2 =
    mismatch(view1, view2);
  good[2] = ( mism == index_of(result2.first) );
  good[3] = ( mism == index_of(result2.second) );

  STAPL_TEST_REPORT(good[0] && good[1] && good[2] && good[3],
                    "Testing mismatch")

  return EXIT_SUCCESS;
}

