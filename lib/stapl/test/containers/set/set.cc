/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/set/set.hpp>
#include "../../test_report.hpp"

using namespace stapl;

struct test_wf
{
  typedef void result_type;

  template<typename T1, typename T2>
  void operator()(T1 const& t, T2 const& nval) const
  {
    //Do something, but can't touch the set element value since it's const.
  }
};

template<typename Set>
void test_set(Set& s, size_t n, bool rebalance=false)
{
  size_t block = n / get_num_locations();
  size_t start = block * get_location_id();
  size_t end = start + block;

  for (size_t i = start; i != end; ++i) {
    s.insert(i);
  }

  rmi_fence();

  STAPL_TEST_REPORT(true, "Testing insert");

  if (rebalance)
  {
    s.rebalance();
    STAPL_TEST_REPORT(s.size() == n, "Testing rebalance");
  }

  STAPL_TEST_REPORT(s.size() == n, "Testing size");

  bool passed = true;

  for (size_t i = 0; i != n; ++i) {
    if (s[i] != i)
      passed = false;
  }

  STAPL_TEST_REPORT(passed, "Testing operator[]");

  //SET_INSERT_FN Uncomment if a use case is found.
  #if 0
  for (size_t i = start; i != end; ++i)
    s.insert(i, test_wf());

  rmi_fence();

  STAPL_TEST_REPORT(s[0]!=s[1], "Testing insert functor");
  #else
  rmi_fence();
  #endif

  s.clear();

  size_t cleared_size = s.size();
  rmi_fence();

  for (size_t i = start; i != end; ++i) {
    s.insert(i);
  }
  rmi_fence();

  size_t count = 0;
  typename Set::iterator it = s.begin();
  while (it != s.end()) {
    ++count;
    ++it;
  }
  rmi_fence();
  STAPL_TEST_REPORT(count == n, "Testing begin/end");

  STAPL_TEST_REPORT(cleared_size == 0, "Testing clear");
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  const size_t n = atol(argv[1]);

  typedef stapl::set<size_t>             container_type;
  typedef indexed_domain<size_t>            domain_type;

  domain_type dom(0, n-1);

  container_type s1(dom);
  STAPL_TEST_REPORT(true, "Testing set(domain) constructor");
  test_set(s1, n);

  container_type s2;
  STAPL_TEST_REPORT(true, "Testing set() constructor");
  test_set(s2, n);

  container_type s3(true);
  STAPL_TEST_REPORT(true, "Testing set() rebalance");
  test_set(s3, n, true);

  domain_type cid_dom(0, get_num_locations()-1);
  stapl::mapper<size_t> mymapper(cid_dom);
  stapl::balanced_partition<domain_type> mypart(dom, get_num_locations());
  container_type dh(mypart, mymapper);
  STAPL_TEST_REPORT(true, "Testing set(Partition, Mapper) constructor");
  test_set(dh, n);

  container_type dh2(mypart);
  STAPL_TEST_REPORT(true, "Testing set(Partitioner) constructor");
  test_set(dh2, n);

  return EXIT_SUCCESS;
}
