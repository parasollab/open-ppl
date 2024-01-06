/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/numeric.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/strided_view.hpp>

#include "../../../test_report.hpp"
#include "../utils.hpp"

using namespace stapl;

template<typename GID, int N>
struct shift_left;

template<typename GID, int N>
struct shift_right
{
  typedef GID gid_type;
  typedef GID index_type;

  typedef shift_left<GID, N> inverse;
  typedef std::true_type is_bijective;

  shift_right(void) = default;
  template<typename T> shift_right(T&&) { }

  gid_type operator()(index_type const& x) const
  {
    return x+N;
  }
};


template<typename GID, int N>
struct shift_left
{
  typedef GID gid_type;
  typedef GID index_type;
  typedef std::true_type is_bijective;

  typedef shift_right<GID, N> inverse;

  shift_left(void) = default;
  template<typename T> shift_left(T&&) { }

  gid_type operator()(index_type const& x) const
  {
    return x-N;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: exe n" << std::endl;
    exit(1);
  }

  size_t n = atoi(argv[1]);

  typedef std::string                          value_t;
  typedef array<value_t>                       array_t;
  typedef array_t::domain_type                 domain_t;
  typedef shift_left<std::size_t, 2>           mf_t;
  typedef array_view<array_t, domain_t, mf_t>  view_t;

  array_t a(n);
  domain_t dom(2, n-3);
  view_t v(a, dom);

  auto c = coarsen_views(v);
  auto& coarsened = get<0>(c);

  bool passed = coarsening_covers_space(v, coarsened);
  STAPL_TEST_REPORT(passed, "Testing invertible projection");

  return EXIT_SUCCESS;
}
