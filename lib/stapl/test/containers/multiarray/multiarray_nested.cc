/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/array/array.hpp>

#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/array_view.hpp>

#include <stapl/algorithms/algorithm.hpp>

#include <test/algorithms/test_utils.h>

#include "../../test_report.hpp"

using namespace stapl;


struct assigner_wf
{
  typedef void result_type;

  template<typename Reference>
  void operator()(Reference r) const
  {
    r = 50 + rand() % 50;
  }
};


struct inner_assigner_wf
{
  typedef void result_type;

  template<typename Reference>
  void operator()(Reference r) const
  {
    r = 1;
  }
};


struct outer_assigner_wf
{
  typedef void result_type;

  template<typename View>
  void operator()(View v) const
  {
    map_func(inner_assigner_wf(), v);
  }
};


struct outer_assigner_ma_wf
{
  typedef void result_type;

  template<typename View>
  void operator()(View v) const
  {
    auto linear_vw = linear_view(v);

    map_func(inner_assigner_wf(), linear_vw);
  }
};


struct accumulate_wf
{
  typedef size_t result_type;

  template<typename V>
  result_type operator()(V v) const
  {
    return accumulate(v, 0);
  }
};


struct accumulate_ma_wf
{
  typedef size_t result_type;

  template<typename View>
  result_type operator()(View v) const
  {
    auto linear_vw = linear_view(v);

    return accumulate(linear_vw, 0);
  }
};


struct tuple_rand_init_wf
{
  typedef size_t result_type;

  template<typename TupleReference>
  result_type operator()(TupleReference r) const
  {
    const size_t x = 16 + rand() % 16;
    const size_t y = 16 + rand() % 16;
    const size_t z = 16 + rand() % 16;

    r = make_tuple(x, y, z);

    return x*y*z;
  }
};


void test_multi3_arr_size_t(void)
{
  //
  // Initialize length view, and compute total number of elements.
  //
  typedef multiarray<3, size_t>                          sizes_ct_t;
  typedef multiarray_view<sizes_ct_t>                    sizes_vw_t;

  sizes_ct_t sizes_ct(make_tuple(8, 8, 16));
  sizes_vw_t sizes_vw(sizes_ct);

  auto linear_sizes_vw = linear_view(sizes_vw);

  map_func(assigner_wf(), linear_sizes_vw);

  const size_t num_elements = accumulate(linear_sizes_vw, 0);

  //
  // Construct and test multiarray<3, array<size_t> >
  //
  typedef multiarray<3, array<size_t> >                  nested_ct_t;
  typedef multiarray_view<nested_ct_t>                   nested_vw_t;

  nested_ct_t nested_ct(sizes_vw);
  nested_vw_t nested_vw(nested_ct);

  auto linear_nested_vw = linear_view(nested_vw);

  map_func(outer_assigner_wf(), linear_nested_vw);

  const size_t computed_sum =
    map_reduce(accumulate_wf(), stapl::plus<size_t>(), linear_nested_vw);

  STAPL_TEST_REPORT(num_elements == computed_sum,
    "Testing multiarray<3, array<size_t> >");

  stapl::rmi_fence();
}


void test_arr_mult3_size_t(void)
{
  typedef array<tuple<size_t, size_t, size_t> >  sizes_ct_t;
  typedef array_view<sizes_ct_t>                 sizes_vw_t;

  sizes_ct_t sizes_ct(32);
  sizes_vw_t sizes_vw(sizes_ct);

  const size_t num_elements =
    map_reduce(tuple_rand_init_wf(), stapl::plus<size_t>(), sizes_vw);

  //
  // Construct and test array<multiarray<3, size_t>
  //
  typedef array<multiarray<3, size_t> >          nested_ct_t;
  typedef array_view<nested_ct_t>                nested_vw_t;

  nested_ct_t nested_ct(sizes_vw);
  nested_vw_t nested_vw(nested_ct);

  map_func(outer_assigner_ma_wf(), nested_vw);

  const size_t computed_sum =
    map_reduce(accumulate_ma_wf(), stapl::plus<size_t>(), nested_vw);

  STAPL_TEST_REPORT(num_elements == computed_sum,
    "Testing array<multiarray<3, size_t> >");

  stapl::rmi_fence();
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  test_multi3_arr_size_t();

  test_arr_mult3_size_t();

  return EXIT_SUCCESS;
}
