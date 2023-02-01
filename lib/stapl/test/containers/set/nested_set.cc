/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/map/map.hpp>
#include <stapl/containers/set/set.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../../test_report.hpp"

using namespace stapl;

struct my_inner_wf
{
  typedef size_t result_type;

  template<typename Element>
  result_type operator()(Element elem) const
  {
    return elem;
  }
};


struct my_outer_wf
{
  typedef size_t result_type;

  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(my_inner_wf(), stapl::plus<size_t>(), elem.second);
  }
};


template<typename View>
struct initialize_wf
{
  typedef void result_type;

  void operator()(View& vw, size_t m_composed_size) const
  {
    size_t idx = 0;

    for (size_t i = 0; i < m_composed_size; ++i)
      for (size_t j = 0; j < m_composed_size; ++j)
        vw[i].insert(++idx);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef indexed_domain<size_t>             domain_type;
  typedef map<size_t, stapl::set<size_t> >   map_set_ct_t;
  typedef map_view<map_set_ct_t>             map_vw_t;
  const size_t n = 1000;

  domain_type dom(0, n-1);
  map_set_ct_t nmap(dom);
  map_vw_t     map_vw(nmap);

  const size_t composed_size   = 100;
  const size_t composed_square = composed_size * composed_size;

  do_once(initialize_wf<map_vw_t>(), map_vw, composed_size);

  const size_t sum =
    stapl::map_reduce(my_outer_wf(), stapl::plus<size_t>(), map_vw);

  const size_t valid_sum = composed_square * (composed_square + 1)  / 2;

  STAPL_TEST_REPORT(sum == valid_sum,
    "Testing simple map<size_t, set<size_t> > nested container");

  return EXIT_SUCCESS;
}
