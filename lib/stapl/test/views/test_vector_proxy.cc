/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include <stapl/views/proxy_macros.hpp>

#include "../test_report.hpp"

using namespace stapl;

struct clear
{
  typedef void result_type;

  template<typename T>
  void operator()(T v)
  {
    for (size_t i = 0; i < v.size(); ++i)
      v[i] = 0;
  }
};


struct verify
{
  typedef bool result_type;

  template<typename T>
  bool operator()(T v)
  {
    for (size_t i = 0; i < v.size(); ++i)
      if (v[i] != size_t(0))
        return false;
    return true;
  }
};


struct verify_resize
{
  typedef bool result_type;

  template<typename T>
  bool operator()(T v)
  {
    v.resize(0);

    if (v.size() != 0)
      return false;

    v.resize(10, 5);

    for (auto&& elem : v)
      if (elem != 5)
        return false;

    return true;
  }
};


class boxed_number
{
  int m_x;

public:
  void set(int x)
  {
    m_x = x;
  }

  int get() const
  {
    return m_x;
  }

  int plus(int a, int b) const
  {
    return m_x + a + b;
  }

  void define_type(typer& t)
  {
    t.member(m_x);
  }
};


namespace stapl {

STAPL_PROXY_HEADER(boxed_number)
{
  STAPL_PROXY_DEFINES(boxed_number)

  STAPL_PROXY_METHOD_RETURN(get, int)
  STAPL_PROXY_METHOD(set, int)
  STAPL_PROXY_METHOD_RETURN(plus, int, int, int)
};

}


void global(size_t n)
{
  typedef std::vector<boxed_number> value_type;
  typedef array<value_type>         array_type;
  typedef array_view<array_type>    view_type;

  array_type a(n, value_type(n));
  view_type v(a);

  if (get_location_id() == get_num_locations() - 1) {
    a[0][0].set(n);
  }

  rmi_fence();

  bool passed = static_cast<size_t>(a[0][0].get()) == n;

  STAPL_TEST_REPORT(passed, "Nested proxies with non-local accessors");
}


void local(size_t n)
{
  typedef std::vector<size_t>    value_type;
  typedef array<value_type>      array_type;
  typedef array_view<array_type> view_type;

  array_type a(n, value_type(n, n));
  view_type v(a);

  map_func(clear(), v);

  bool passed = map_reduce(verify(), stapl::logical_and<bool>(), v);

  STAPL_TEST_REPORT(passed, "Nested proxies with non-local accessors");
}


void local_resize(size_t n)
{
  typedef std::vector<size_t>    value_type;
  typedef array<value_type>      array_type;
  typedef array_view<array_type> view_type;

  array_type a(n, value_type(n, n));
  view_type v(a);

  map_func(clear(), v);

  bool passed = map_reduce(verify_resize(), stapl::logical_and<bool>(), v);

  STAPL_TEST_REPORT(passed, "vector proxy resize meethod");
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  size_t n = atoi(argv[1]);

  global(n);

  local(n);

  local_resize(n);

  return EXIT_SUCCESS;
}
