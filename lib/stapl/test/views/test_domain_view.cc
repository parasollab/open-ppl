/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/containers/array/array.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/numeric.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/domain_view.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>

#include "../test_report.hpp"

using namespace stapl;


struct print_wf
{
  template <typename Ref>
  void operator()(Ref x) const { std::cout << x << " "; }
};

template <typename T>
struct map_wf
{
  typedef size_t result_type;

  T m_val;
  map_wf(const T& v) : m_val(v) {}

  template <typename Ref, typename Idx>
  size_t operator()(Ref ref, Idx idx)
  { T d = ref;
    return ((m_val==d) ? idx : index_bounds<size_t>::highest());
  }

  void define_type(stapl::typer &t)
  { t.member(m_val); }
};

template <typename T>
struct red_wf
{
  typedef size_t result_type;
  template <typename T0, typename T1>
  T operator()(T0 i0, T1 i1)
  {
    return ((i0!=index_bounds<size_t>::highest()) ? i0 : i1);
  }
};


template <typename View>
typename View::index_type my_find(const View& vw, typename View::value_type val)
{
  return  map_reduce(map_wf<typename View::value_type>(val),
                     red_wf<typename View::index_type>(),
                     vw,stapl::domain_view(vw));
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef array<size_t>            pa_t;
  typedef array_view<pa_t>         view_t;
  typedef view_t::domain_type      vec_dom_t;

  size_t n = 100;

  pa_t pa1(n);

  view_t view1(pa1);

  size_t res = accumulate(stapl::domain_view(view1),0);

  size_t m = n*(n-1)/2;
  STAPL_TEST_REPORT(res == m,"Testing domain_view [0..99]");


  view_t view4(pa1,vec_dom_t(10,n-11));

  res = accumulate(stapl::domain_view(view4),0);

  m = ((n-10)*(n-11)/2)-(45);
  STAPL_TEST_REPORT(res == m,"Testing domain_view [10..89]");


  copy(counting_view<size_t>(view4.size()),view4);

  size_t idx = my_find(view1,7);
  STAPL_TEST_REPORT(idx == 17,"Testing domain_view (find)");

  return EXIT_SUCCESS;
}
