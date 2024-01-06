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
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/counting_view.hpp>

#include "../../test_report.hpp"

using namespace stapl;

struct pred
{
  typedef bool result_type;

  result_type m_result;

  pred()
    : m_result(false)
  { }

  template<typename T>
  result_type operator()(T t) const
  {
    return t % 2 == 0;
  }

  void define_type(typer& t)
  {
    t.member(m_result);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef array<int> p_int_type;
  typedef array_view<p_int_type> pintView;

  stapl::counter<stapl::default_timer> timer;

  int nelem = 1000;

  p_int_type pint1(nelem);
  p_int_type pint2(nelem);
  p_int_type pint3(nelem);

  pintView pintv1(pint1);
  pintView pintv2(pint2);
  pintView pintv3(pint3);

  copy(counting_view<int> (nelem, 0), pintv1);

  timer.reset();
  timer.start();

  std::pair<pintView, pintView> x =
    partition_copy(pintv1, pintv2, pintv3, pred());
  double exec_time = timer.stop();

  unsigned int i = 0;
  unsigned int j = 0;
  bool result = true;

  if (get_location_id() == 0) {
    std::cout << std::endl;
    while (i < x.first.size() && result) {
      if (x.first[i] != pint1[j]) {
        result = false;
      }
      i++;
      j=j+2;
    }
    j = 1;
    i = 0;
    while (i < x.second.size() && result) {
      if (x.second[i] != pint1[j]) {
        result = false;
      }
      i++;
      j=j+2;
    }
  }

  stapl::do_once([&exec_time, &result]() {
    printf("Test: partition_copy\nStatus: ");
    if (result)
      printf("PASS");
    else
      printf("FAIL");
    printf("\nVersion: stapl\nTime: %f\n", exec_time);
  });

  return EXIT_SUCCESS;
}
