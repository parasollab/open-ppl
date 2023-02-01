/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/array/array.hpp>
#include <stapl/containers/array/proxy.hpp>
#include <stapl/containers/type_traits/container_levels.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include "../../test_report.hpp"
#include <boost/mpl/assert.hpp>

using namespace stapl;

class resize_wf
{
  size_t m_size;

public:
  typedef void result_type;

  resize_wf(size_t const& size)
    : m_size(size)
  { }

  template<typename T>
  void operator()(T a)
  {
    a.resize(m_size);
  }

  void define_type(typer& t)
  {
    t.member(m_size);
  }
};


class size_wf
{
  size_t m_size;

public:
  size_wf(size_t const& size)
    : m_size(size)
  { }

  typedef bool result_type;

  template<typename T>
  bool operator()(T a)
  {
    return a.size() == m_size;
  }

  void define_type(typer& t)
  {
    t.member(m_size);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cerr << "usage: exe n m" << std::endl;
    exit(1);
  }

  const size_t n = atoi(argv[1]);

  if (n < 5)
    abort("n needs to be at least 5");

  const size_t m = atoi(argv[2]);

  typedef array<array<int> > array_type;

  BOOST_MPL_ASSERT((stapl::is_container<array_type>));
  BOOST_MPL_ASSERT_RELATION(stapl::container_levels<array_type>::value, ==, 2);

  array_type c(n);

  typedef array_view<array_type> view_type;
  view_type v(c);
  stapl::map_func(resize_wf(m), v);

  const bool b_not_null = !is_null_reference(v[0]);

  STAPL_TEST_REPORT(b_not_null, "Testing is_null_reference")

  const bool b_correct_index = index_of(v[5]) == 5;

  STAPL_TEST_REPORT(b_correct_index, "Testing index_of")

  return EXIT_SUCCESS;
}
