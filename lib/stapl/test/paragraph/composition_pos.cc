/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>

#include <cstdlib>
#include <iostream>

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/skeletons/explicit/new_map.h>

#include <test/algorithms/test_utils.h>

int x = 0;

struct my_ident
{
  typedef int result_type;

  int m_val;

  void define_type(stapl::typer& t)
  {
    t.member(m_val);
  }

  my_ident()
    : m_val(0)
  { }

  template<typename Reference>
  int operator()(Reference elem)
  {
    return elem + (++m_val);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef stapl::array<int>                                               ct_t;
  typedef stapl::array_view<ct_t>                                         vw_t;
  typedef stapl::composition::result_of::map_func<my_ident, vw_t>::type   res_t;

  using stapl::fill;

  if (stapl::get_location_id() == 0)
    std::cout << "paragraph composition positive with "
              << stapl::get_num_locations() << " locations... ";

  const size_t nelems = stapl::get_num_locations() * 100;
  const size_t offset = stapl::get_location_id()   * 100;

  // Container and View Construction
  ct_t ct1(nelems);
  vw_t vw1(ct1);

  fill(vw1, stapl::get_location_id());

  res_t ret_vw1 = stapl::composition::map_func(my_ident(), vw1);

  res_t ret_vw2 = stapl::composition::map_func(my_ident(), ret_vw1);

  res_t ret_vw3 = stapl::composition::map_func(my_ident(), ret_vw2);

  bool error = ret_vw3.size() != nelems;

  for (int i=0; i < 100; ++i)
    if (ret_vw3[i + offset] != (int) ((i+1)*3 + stapl::get_location_id()))
      error = true;

  stapl::stapl_bool test_result(error);
  test_result.reduce();

  if (stapl::get_location_id() == 0)
  {
    if (test_result.value() == false)
      std::cout << "Passed\n";
    else
      std:: cout << "Failed\n";
  }

  return EXIT_SUCCESS;
}
