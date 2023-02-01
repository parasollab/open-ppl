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

#include "../test_report.hpp"

using namespace stapl;

template<typename T>
struct setter
{
private:
  typename T::first_type m_first;
  typename T::second_type m_second;

public:
  typedef T result_type;

  setter() = default;

  setter(typename T::first_type first, typename T::second_type second)
    : m_first(first), m_second(second)
  { }

  template<typename P>
  result_type operator()(P p)
  {
    p.first = m_first;
    p.second = m_second;
    return p;
  }

  void define_type(typer& t)
  {
    t.member(m_first);
    t.member(m_second);
  }
};


template<typename T>
struct checker
{
  typename T::first_type m_first;
  typename T::second_type m_second;

  typedef bool result_type;
  checker() = default;

  checker(typename T::first_type first, typename T::second_type second)
    : m_first(first), m_second(second)
  { }

  template<typename P>
  result_type operator()(P x)
  {
    return x.first == m_first && x.second == m_second;
  }

  void define_type(typer& t)
  {
    t.member(m_first);
    t.member(m_second);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  size_t n = atoi(argv[1]);

  typedef std::pair<long, bool>     pair_type;
  typedef array<pair_type>          array_type;
  typedef array_view<array_type>    view_type;

  array_type a(n);
  view_type v(a);

  map_func(setter<pair_type>(n, true), v);
  size_t count = count_if(v, checker<pair_type>(n, true));

  STAPL_TEST_REPORT(count == n, "Testing specialization of proxy for std::pair")

  return EXIT_SUCCESS;
}
