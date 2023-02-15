/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/algorithms/algorithm.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>

#include "../../test_report.hpp"

using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef array<char>                p_array_char;
  typedef array_view<p_array_char>   view_array_char;

  int minelem =0;
  if (argc == 2)
    minelem = atoi(argv[1]);
  else
    minelem = 10;

  if (minelem < 5) minelem = 5;

  p_array_char p_array1(minelem);
  p_array_char p_array2(minelem);
  p_array_char p_array3(minelem);
  p_array_char p_array4(minelem);
  view_array_char pview1(p_array1);
  view_array_char pview2(p_array2);
  view_array_char pview3(p_array3);
  view_array_char pview4(p_array4);

  fill(pview1,'x');

  pview1[0]='a'; pview1[1]='b'; pview1[2]='c'; pview1[3]='d'; pview1[4]='e';
  pview2[0]='f'; pview2[1]='g'; pview2[2]='h'; pview2[3]='c';
  pview3[0]='f'; pview3[1]='g'; pview3[2]='h'; pview3[3]='b'; pview3[4]='e';
  pview4[0]='f'; pview4[1]='g';

  view_array_char::reference result1 =
    find_first_of(pview1,pview2);
  view_array_char::reference result2 =
    find_first_of(pview1,pview3);
  view_array_char::reference result3 =
    find_first_of(pview1,pview4);

  bool res = (result1 =='c' && result2 =='b' && is_null_reference(result3));

  STAPL_TEST_REPORT(res,"Testing find_first_of");

  return EXIT_SUCCESS;
}
