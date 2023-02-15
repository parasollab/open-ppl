/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/map/map.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../../test_report.hpp"

using namespace stapl;

struct test_wf
{
  typedef void result_type;

  template<typename T1, typename T2>
  void operator()(T1& t, T2 const& nval) const
  {
    ++t.second;
  }
};

struct my_inner_wf
{
  typedef size_t result_type;

  template<typename Element>
  result_type operator()(Element elem) const
  {
    return elem.second;
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


struct set_to_x
{
  typedef void result_type;

  template<typename T>
  void operator()(T& t) const
  {
    t = 'x';
  }
};


struct increment
{
  typedef char result_type;

  template<typename T>
  result_type operator()(T& t) const
  {
    return t + 1;
  }
};


template<typename Map>
void test_map(Map& m, size_t n)
{
  size_t block = n / get_num_locations();
  size_t start = block * get_location_id();
  size_t end = start + block;

  for (size_t i = start; i != end; ++i)
    m.insert(i, static_cast<char>(i % std::numeric_limits<char>::max()));

  rmi_fence();

  STAPL_TEST_REPORT(m.size() == n, "Testing insert and size");

  bool passed = true;

  for (size_t i = 0; i != n; ++i)
    if (m[i] != i % std::numeric_limits<char>::max())
      passed = false;

  STAPL_TEST_REPORT(passed, "Testing operator[]");

  for (size_t i = start; i != end; ++i)
    m.insert(i, 'a', test_wf());

  rmi_fence();

  STAPL_TEST_REPORT(m[0]!=m[1], "Testing insert functor");

  m.data_apply_async(0, set_to_x());

  STAPL_TEST_REPORT(m[0] == 'x', "Testing data_apply_async");

  char y = m.data_apply(0, increment());

  STAPL_TEST_REPORT(y == 'y', "Testing data_apply");

  m.clear();

  size_t cleared_size = m.size();

  rmi_fence();

  for (size_t i = start; i != end; ++i)
    m.insert(i, static_cast<char>(i % std::numeric_limits<char>::max()));

  STAPL_TEST_REPORT(cleared_size == 0, "Testing clear");

  STAPL_TEST_REPORT(m.find(0)!=m.end(), "Testing find on existing key");

  STAPL_TEST_REPORT(m.find(n+8)==m.end(), "Testing find on nonexistent key");

  STAPL_TEST_REPORT(m.count(0)==1, "Testing count on existing key");

  STAPL_TEST_REPORT(m.count(n+8)==0, "Testing count on nonexistent key");
}


template<typename Container>
struct initialize_wf
{
  typedef void result_type;

  void operator()(Container& m_ct, size_t m_composed_size) const
  {
    size_t idx = 0;

    for (size_t i = 0; i < m_composed_size; ++i)
      for (size_t j = 0; j < m_composed_size; ++j)
        m_ct[i][j] = ++idx;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  const size_t n = atol(argv[1]);

  typedef stapl::map<size_t, unsigned char> container_type;
  typedef indexed_domain<size_t>            domain_type;

  container_type m1;

  STAPL_TEST_REPORT(true, "Testing map() constructor");

  test_map(m1, n);

  domain_type dom(0, n-1);

  container_type m2(dom);

  STAPL_TEST_REPORT(true, "Testing map(domain) constructor");

  test_map(m2, n);

  //
  // Simple Composed Map Test
  //
  typedef stapl::map<size_t, stapl::map<size_t, size_t > > map_ct_t;
  typedef map_view<map_ct_t>                               map_vw_t;

  map_ct_t nmap(dom);
  map_vw_t map_vw(nmap);

  const size_t composed_size   = 100;
  const size_t composed_square = composed_size * composed_size;

  do_once(initialize_wf<map_vw_t>(), map_vw, composed_size);

  const size_t sum =
    stapl::map_reduce(my_outer_wf(), stapl::plus<size_t>(), map_vw);

  const size_t valid_sum = composed_square * (composed_square + 1)  / 2;

  STAPL_TEST_REPORT(sum == valid_sum,
    "Testing simple map<size_t, map<size_t, size_t> > composed container");

  return EXIT_SUCCESS;
}
