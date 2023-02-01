/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/array.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../test_report.hpp"

using namespace stapl;


typedef std::map<int,int>                 value_type;
typedef array<value_type>                 array_type;
typedef array_view<array_type>            view_type;
typedef array_type::reference::iterator   iterator;


struct init_wf
{
  int m_n;

  typedef void result_type;

  init_wf(int n)
    :m_n(n)
  {}

  template<typename T>
  void operator()(T m)
  {
    for (int i = 1; i <= m_n; ++i)
      m[i]=1;
  }

  void define_type(typer& t) const
  {
    t.member(m_n);
  }
};

template<typename T>
bool verify(T& a, int n)
{
// uncomment when member_iterator supports maps
#if 0
  int key_sum, val_sum;
  for (int i = 0; i < n; ++i){
    iterator it = a[i].begin();
    key_sum = 0;
    val_sum = 0;
    for (; it != a[i].end() ; ++it)
    {
      key_sum += (*it).first;
      val_sum += (*it).second;
    }
    if (val_sum != n)
      return false;
    else if (key_sum != (n*(n+1)/2) )
      return false;
  }
#endif
  return true;
}

void global(int n)
{

  array_type a(n, value_type());
  view_type v(a);

  bool passed = a[0].empty();
  STAPL_TEST_REPORT(passed, "Nested proxies empty");

  if (get_location_id() == get_num_locations() - 1) {
    a[0][10] = n;
    a[0][11] = n;
    a[0][12] = n;
    a[0][13] = n;
  }

  rmi_fence();

  passed = a[0].size() == 4;
  STAPL_TEST_REPORT(passed, "Nested proxies size");

  passed = a[0][10] == n;
  STAPL_TEST_REPORT(passed, "Nested proxies with non-local accessors");

// uncomment when member_iterator supports maps
#if 0
  iterator it;
  int i=10;
  passed = true;
  for (it = a[0].begin(); it != a[0].end() ; ++it , ++i)
  {
    passed = passed & ((*it).first == i);
  }
  STAPL_TEST_REPORT(passed, "Nested proxies begin/end");
#endif
}

void local(int n)
{

  array_type a(n, value_type());
  view_type v(a);

  for_each(v,init_wf(n));

  bool passed = verify(a, n);

  STAPL_TEST_REPORT(passed, "Nested proxies with local accessors");
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  int n = atoi(argv[1]);

  global(n);

  local(n);

  return EXIT_SUCCESS;
}
