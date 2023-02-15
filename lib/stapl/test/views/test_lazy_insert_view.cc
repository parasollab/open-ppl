/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/vector/vector.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include <stapl/views/vector_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/lazy_insert_view.hpp>

#include "../test_report.hpp"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Workfunction to add an element to the lazy insert view for
///        each element of the original view
//////////////////////////////////////////////////////////////////////
struct insert_wf
{
  typedef void result_type;

  template<typename T, typename LazyInsert>
  void operator()(T x, LazyInsert liv)
  {
    liv.add(x+1);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using vector_type = stapl::vector<std::size_t>;
  using view_type = vector_view<vector_type>;

  const std::size_t n = 10;

  vector_type v(n);
  view_type vw(v);

  map_func(insert_wf(), vw, make_repeat_view(lazy_insert(v)));


  const std::size_t new_size = v.size();

  const bool passed = new_size == 2*n;

  STAPL_TEST_REPORT(passed, "Lazy insert view over vector");

  return EXIT_SUCCESS;
}
