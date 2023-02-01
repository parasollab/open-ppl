/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/skeletons/map.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>

#include <stapl/views/array_view.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/repeated_view.hpp>

#include <stapl/algorithms/algorithm.hpp>

using namespace stapl;


struct assign1_wf
{
  typedef void result_type;

  template<typename Ref, typename View>
  void operator()(Ref elem, View v) const
  {
    for (size_t i=0; i<v.size(); ++i)
      elem += v[i];
  }
};


struct assign2_wf
{
  typedef void result_type;

  template<typename View, typename Ref>
  void operator()(View v, Ref elem) const
  {
    for (size_t i=0; i<v.size(); ++i)
      elem += v[i];
  }
};


struct assign3_wf
{
  typedef void result_type;

  template<typename Ref1, typename View1, typename Ref2>
  void operator()(Ref1 elem, View1 v1, Ref2) const
  {
    for (size_t i=0; i<v1.size(); ++i)
      elem += v1[i];
  }
};


struct assign4_wf
{
  typedef void result_type;

  template<typename View1, typename Ref1, typename Ref2>
  void operator()(View1 v1, Ref1 elem, Ref2) const
  {
    for (size_t i=0; i<v1.size(); ++i)
      elem += v1[i];
  }
};


struct assign5_wf
{
  typedef void result_type;

  template<typename View1, typename Ref1, typename Ref2>
  void operator()(Ref1, Ref2 elem, View1 v1) const
  {
    for (size_t i = 0; i < get<0>(v1.dimensions()); ++i)
      for (size_t j = 0; j < get<1>(v1.dimensions()); ++j)
        elem += v1(i,j);
  }
};


struct assign6_wf
{
  typedef void result_type;

  template<typename View1, typename Ref1, typename Ref2>
  void operator()(Ref1 elem, View1 v1, Ref2) const
  {
    for (size_t i = 0; i < get<0>(v1.dimensions()); ++i)
      for (size_t j = 0; j < get<1>(v1.dimensions()); ++j)
        elem += v1(i,j);
  }
};


exit_code stapl_main(int argc, char* argv[])
{
  //
  // array / repeated 1D tests
  //
  do_once([]() { std::cout << "Testing array / repeat_view (1D)..."; });

  typedef array<int>               array_type;
  typedef array_view<array_type>   view_type;

  array_type ar1(100, 1);
  array_type ar2(100, 0);
  array_type ar3(100, 0);
  array_type ar4(100, 0);
  array_type ar5(100, 0);

  view_type v1(ar1);
  view_type v2(ar2);
  view_type v3(ar3);
  view_type v4(ar4);
  view_type v5(ar5);

  map_func(assign1_wf(), v2, make_repeat_view(v1));
  map_func(assign2_wf(), make_repeat_view(v1), v3);
  map_func(assign3_wf(), v4, make_repeat_view(v1), v2);
  map_func(assign4_wf(), make_repeat_view(v1), v5, v2);

  bool ret_val = (count(v2, 100) == 100)
              && (count(v3, 100) == 100)
              && (count(v4, 100) == 100)
              && (count(v5, 100) == 100);


  do_once([ret_val]() {
    if (ret_val)
      std::cout << "Passed\n";
    else
      std::cout << "Fail\n";
  });


  //
  // multiarray / repeated 2D tests
  //
  do_once([]() { std::cout << "Testing multiarray / repeat_view (2D)..."; });

  typedef multiarray<2, int>          ma_ct_type;
  typedef multiarray_view<ma_ct_type> ma_vw_type;

  ma_ct_type ma_ct1(std::make_tuple(10, 10), 1);
  ma_ct_type ma_ct2(std::make_tuple(10, 10), 0);
  ma_ct_type ma_ct3(std::make_tuple(10, 10), 0);

  ma_vw_type ma_vw1(ma_ct1);
  ma_vw_type ma_vw2(ma_ct2);
  ma_vw_type ma_vw3(ma_ct3);

  // NOTE - repeat_view as first view currently fails, probably due to
  // assumptions in volumetric alignment.
  map_func(assign5_wf(), ma_vw2, ma_vw3, make_repeat_view_nd<2>(ma_vw1));
  map_func(assign6_wf(), ma_vw2, make_repeat_view_nd<2>(ma_vw1), ma_vw3);

  ret_val = (count(linear_view(ma_vw2), 100) == 100)
         && (count(linear_view(ma_vw3), 100) == 100);

  do_once([ret_val]() {
    if (ret_val)
      std::cout << "Passed\n";
    else
      std::cout << "Fail\n";
  });


  //
  // mixed / repeated 2D tests
  //
  do_once([]() { std::cout << "Testing mixed / repeat_view (2D)..."; });

  ma_ct_type ma_ct4(std::make_tuple(10, 10), 0);
  ma_vw_type ma_vw4(ma_ct4);

  map_func(assign1_wf(), ma_vw4, make_repeat_view_nd<2>(v1));

  ret_val = (count(linear_view(ma_vw4), 100) == 100);

  do_once([ret_val]() {
    if (ret_val)
      std::cout << "Passed\n";
    else
      std::cout << "Fail\n";
  });

  return EXIT_SUCCESS;
}
