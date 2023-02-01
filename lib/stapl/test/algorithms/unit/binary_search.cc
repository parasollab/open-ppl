/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <algorithm>

#include <stapl/runtime.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/views/counting_view.hpp>

#include "../../test_report.hpp"

using namespace stapl;

template< typename T >
struct strided_trans_wf
{
private:
  T m_stride;
public:
  strided_trans_wf(T t) : m_stride(t) {}

  void define_type(typer& t)
  {
    t.member(m_stride);
  }

  typedef void result_type;

  template < typename V >
  void operator()(V&& t)
  {
    t = (t / m_stride) + 1;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::counter<stapl::default_timer> timer;
  int num_elem = 100;
  double exec_time;

  if (argc >= 2)
    num_elem = atoi(argv[1]);

  //Force a boundary condition on the numbers generated
  int range_size = (num_elem / (get_num_locations() + 10));
  range_size = (range_size > 0) ? range_size : 1;

  typedef array<int> p_array_type;
  typedef array_view<p_array_type> view_type;
  typedef view_type::reference ref_t;
  typedef result_of::counting_view<int>::type count_t;

  p_array_type array(num_elem);
  view_type view(array);
  count_t count = counting_view(num_elem,1);

  copy(count, view);
  for_each(view, strided_trans_wf<int>(range_size));

  int end = (view[view.domain().last()] + 1);

  bool res = true;
  for (int i = view[view.domain().first()]; i <= end; i++)
  {
    timer.start();
    bool result = binary_search(view, i);
    timer.stop();
    do_once([&](void) {
      res = res & (result == std::binary_search(view.begin(), view.end(), i));
    } );
  }

  exec_time = timer.value() / timer.calls();
  stapl::do_once([&exec_time, &res](void) {
    printf("Test: binary_search\nStatus: ");
    if (res)
      printf("PASS");
    else
      printf("FAIL");
    printf("\nVersion: stapl\nTime: %f\n\n", exec_time);
  });
  timer.reset();

  res = true;
  for (int i = view[view.domain().first()]; i <= end; i++)
  {
    timer.start();
    ref_t lower_res = lower_bound(view, i);
    timer.stop();
    view_type::iterator iter;
    do_once([&](void) {
      iter = std::lower_bound(view.begin(), view.end(), i);
      if (iter == view.end() && is_null_reference(lower_res))
      {
        res = res & true;
      }
      else
      {
        if (index_of(lower_res) == index_of(*iter))
        {
          res = res & true;
        }
        else
        {
          res = res & false;
        }
      }
    });
  }

  exec_time = timer.value() / timer.calls();
  stapl::do_once([&exec_time, &res](void) {
    printf("Test: lower_bound\nStatus: ");
    if (res)
      printf("PASS");
    else
      printf("FAIL");
    printf("\nVersion: stapl\nTime: %f\n\n", exec_time);
  });
  timer.reset();

  res = true;
  for (int i = view[view.domain().first()]; i <= end; i++)
  {
    timer.start();
    ref_t upper_res = upper_bound(view, i);
    timer.stop();
    view_type::iterator iter;
    do_once([&](void) {
      iter = std::upper_bound(view.begin(), view.end(), i);
      if (iter == view.end() && is_null_reference(upper_res))
      {
        res = res & true;
      }
      else
      {
        if (index_of(upper_res) == index_of(*iter))
        {
          res = res & true;
        }
        else
        {
          res = res & false;
        }
      }
    });
  }

  exec_time = timer.value() / timer.calls();
  stapl::do_once([&exec_time, &res](void) {
    printf("Test: upper_bound\nStatus: ");
    if (res)
      printf("PASS");
    else
      printf("FAIL");
    printf("\nVersion: stapl\nTime: %f\n\n", exec_time);
  });
  timer.reset();

  res = true;

  typedef std::pair<view_type::iterator, view_type::iterator> pair_iter_t;

  for (int i = view[view.domain().first()]; i <= end; i++)
  {
    timer.start();
    view_type equal_res = equal_range(view, i);
    timer.stop();
    pair_iter_t iter;
    do_once([&](void) {
      iter = std::equal_range(view.begin(), view.end(), i);
      res &= std::equal(iter.first, iter.second, equal_res.begin());
    });
  }

  exec_time = timer.value() / timer.calls();
  stapl::do_once([&exec_time, &res](void) {
    printf("Test: equal_range\nStatus: ");
    if (res)
      printf("PASS");
    else
      printf("FAIL");
    printf("\nVersion: stapl\nTime: %f\n\n", exec_time);
  });


  return EXIT_SUCCESS;
}
