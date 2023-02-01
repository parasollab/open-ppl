/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <vector>
#include <list>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/list_view.hpp>
#include <stapl/utility/do_once.hpp>

#include "../test_report.hpp"


using namespace stapl;


struct equal_one
{
  typedef size_t result_type;

  template<typename Ref>
  size_t operator()(Ref r)
  {
    return r == 1;
  }
};


struct nested_mr
{
  typedef size_t result_type;

  template<typename VectorRef>
  size_t operator()(VectorRef& vec_ref)
  {
    return map_reduce<skeletons::tags::no_coarsening>(
      equal_one(), stapl::plus<size_t>(), make_array_view(vec_ref));
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  // Testing view over std:vector

  typedef std::vector<size_t>        pa_t;
  typedef array_view<pa_t>           view_t;
  typedef view_t::domain_type        dom_t;

  pa_t pa1(100,1);

  view_t view1(pa1);
  size_t res = count(view1,size_t(1));
  STAPL_TEST_REPORT(res == 100,"Testing std::vector [0..99]");

  view_t view4(pa1,dom_t(10,89));
  res = count(view4,size_t(1));
  STAPL_TEST_REPORT(res == 80,"Testing std::vector [10..89]");


  // Testing view over std:list

  typedef std::list<size_t>          pb_t;
  typedef list_view<pb_t, dom1D<pb_t::iterator, seqDom<pb_t::iterator> > >
    viewb_t;
  typedef viewb_t::domain_type       domb_t;

  pb_t pb1(100,1);

  viewb_t view1b(pb1, domb_t(pb1.begin(), --pb1.end()));
  res = count(view1b,size_t(1));
  STAPL_TEST_REPORT(res == 100,"Testing std::list [0..99]");

  pb_t::iterator low = pb1.begin();
  pb_t::iterator upr = pb1.begin();

  // pb_t::iterator is not a random iterator
  for (size_t i=0;i<10;++i)
    ++low;

  for (size_t i=0;i<60;++i)
    ++upr;

  viewb_t view4b(pb1,domb_t(low,upr));
  res = count(view4b,size_t(1));
  STAPL_TEST_REPORT(res == 51,"Testing std::list [10..60]");


  // Testing view<proxy<vector<T>, A>> (non coarsened).

  array<pa_t> ct(10, pa_t(10, 1));

  res = map_reduce<skeletons::tags::no_coarsening>(
    nested_mr(), stapl::plus<size_t>(), make_array_view(ct));

  STAPL_TEST_REPORT(res == 100,
    "Testing view<proxy<vector<T>, A>> (non coarsened)");

  return EXIT_SUCCESS;
}
