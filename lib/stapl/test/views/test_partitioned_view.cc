/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <string>

#include <stapl/algorithm.hpp>
#include <stapl/numeric.hpp>
#include <stapl/array.hpp>
#include <stapl/list.hpp>

#include <stapl/views/balance_view.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/views/overlap_view.hpp>
#include <stapl/utility/do_once.hpp>

#include "../test_report.hpp"

using namespace stapl;

struct get_size
{
  typedef int result_type;
  template<typename View>
  int operator()(View v) const
  {
    return v.size();
  }
};


// mapping function that avoids specializations for identity.
struct unspecialized_identity
{
  typedef size_t gid_type;
  typedef size_t index_type;

  size_t operator()(size_t id) const
  { return id; }
};


struct test_overlap_wf
{
  typedef bool result_type;

  template <typename OverlapView, typename Index>
  bool operator()(OverlapView xv, Index i) const
  {
    int neighbor_value = xv[xv.domain().first()];

    if (i == 0)
      return xv.size() == 1 && neighbor_value == i;

    return xv.size() == 2 && neighbor_value == i-1 &&
      xv[xv.domain().first()+1] == i;
  }
};

template<typename View>
void test_native_view(View& vw, size_t n, std::string str)
{
  size_t res =
    map_reduce(get_size(), stapl::plus<int>(), stapl::native_view(vw));

  STAPL_TEST_REPORT(res == n, str);
}


template<typename View>
void test_overlap_view(View& vw, size_t n, std::string str)
{
  size_t res = map_reduce(get_size(), stapl::plus<int>(),
                          stapl::make_overlap_view(vw,1,0,2));
  STAPL_TEST_REPORT(res == (n-2)*3 + 2 + 1, str);
}


template<typename View>
void test_overlap_view_nonid(View& vw, size_t n, std::string str)
{
  stapl::array_view<typename View::view_container_type,
    stapl::indexed_domain<size_t>, unspecialized_identity>
  vw_nonid(vw.container(), stapl::indexed_domain<size_t>(n),
    unspecialized_identity());

  auto vw_nonid_o = stapl::make_overlap_view(vw_nonid, 1, 1, 0);

  bool result = stapl::map_reduce(test_overlap_wf(), stapl::logical_or<bool>(),
                  vw_nonid_o, stapl::counting_view<int>(n, 0));
  STAPL_TEST_REPORT(result,
    "Testing overlap_view with non-identity mapping function: ")
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t n = 100;

  if (argc < 2) {
    std::cout << "usage: "<< argv[0] <<" n" << std::endl;
    exit(0);
  }
  else
    n = atoi(argv[1]);

  list<int> pl(n);
  typedef list_view<list<int> > lview_t;
  lview_t lv(pl);
  stapl::iota(lv, 0);

  test_native_view(lv,n,"Testing native_partitioned_view over list_view<list>");
  test_overlap_view(lv,n,"Testing overlap_view over list_view<list>");

  lview_t::domain_type rest_dom(lv.domain().first(),
                                lv.domain().advance(lv.domain().first(),n/3-1),
                                lv.domain());
  lview_t lv1_3(pl,rest_dom);
  test_native_view(lv1_3,n/3,
           "Testing native_partitioned_view over a restricted list_view<list>");


  array<int> pa(n);
  typedef array_view<array<int> > aview_t;
  aview_t av(pa);
  stapl::iota(av, 0);

  test_native_view(av,n,
                   "Testing native_partitioned_view over array_view<array>");
  test_overlap_view(av,n,"Testing overlap_view over array_view<array>");
  test_overlap_view_nonid(av,n,"Testing overlap_view over array_view<array>");

  aview_t av1_3(pa,aview_t::domain_type(n/3,2*n/3-1));
  test_native_view(av1_3, n/3,
       "Testing native_partitioned_view over a restricted array_view<array>");
  aview_t av0_3(pa,aview_t::domain_type(0,n/3-1));
  test_overlap_view(av0_3, n/3,
       "Testing overlap_view over a restricted array_view<array>");

  return EXIT_SUCCESS;
}
