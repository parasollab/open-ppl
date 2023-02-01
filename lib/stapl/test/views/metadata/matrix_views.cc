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
#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/strided_view.hpp>

#include "../../test_report.hpp"
#include "utils.hpp"

using namespace stapl;

struct F0
{
  typedef std::tuple<std::size_t, std::size_t> index_type;
  typedef std::tuple<std::size_t, std::size_t> gid_type;

  F0(void) = default;
  template<typename T> F0(T&&) { }

  gid_type operator()(index_type const& x) const
  {
    return x;
  }
};


struct F1
{
  typedef std::tuple<std::size_t, std::size_t> index_type;
  typedef std::tuple<std::size_t, std::size_t> gid_type;

  F1(void) = default;
  template<typename T> F1(T&&) { }

  gid_type operator()(index_type const& x) const
  {
    return std::make_tuple(0, 0);
  }
};


struct F2
{
  typedef std::tuple<std::size_t, std::size_t> index_type;
  typedef std::tuple<std::size_t, std::size_t> gid_type;

  //gid_type m_size;

  F2(void) = default;
  template<typename T> F2(T&&) { }

  gid_type operator()(index_type const& x) const
  {
    return std::make_tuple(
      get<0>(x) / 2,
      get<1>(x) / 2
    );
  }
};


struct F3
{
  typedef std::tuple<std::size_t, std::size_t> index_type;
  typedef std::tuple<std::size_t, std::size_t> gid_type;

  F3(void) = default;
  template<typename T> F3(T&&) { }

  gid_type operator()(index_type const& x) const
  {
    return std::make_tuple(
      get<0>(x) % 2,
      get<1>(x) % 2
    );
  }
};


struct F4
{
  typedef std::tuple<std::size_t, std::size_t> index_type;
  typedef std::tuple<std::size_t, std::size_t> gid_type;

  F4(void) = default;
  template<typename T> F4(T&&) { }

  gid_type operator()(index_type const& x) const
  {
    return std::make_tuple(
      get<0>(x) % get_num_locations(),
      get<1>(x) % get_num_locations()
    );
  }
};


template<typename MF>
bool test_matrix_view_coarsening(std::size_t n, MF const& mf = MF())
{
  typedef std::string                                       value_type;
  typedef multiarray<2, value_type>                         multiarray_type;
  typedef multiarray_type::domain_type                      domain_type;
  typedef multiarray_view<multiarray_type, domain_type, MF> view_type;

  multiarray_type a(std::make_tuple(n, n));
  view_type v(a);

  auto c = coarsen_views(v);
  auto& coarsened = get<0>(c);

  return coarsening_covers_space(v, coarsened);
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: exe n" << std::endl;
    exit(1);
  }

  typedef std::tuple<std::size_t, std::size_t> gid_type;

  const std::size_t n = atoi(argv[1]);

  bool passed;

  passed = test_matrix_view_coarsening<F0>(n);
  STAPL_TEST_REPORT(passed, "Testing view with F0");

  passed = test_matrix_view_coarsening<F1>(n);
  STAPL_TEST_REPORT(passed, "Testing view with F1");

  passed = test_matrix_view_coarsening<F2>(n);
  STAPL_TEST_REPORT(passed, "Testing view with F2");

  passed = test_matrix_view_coarsening<F3>(n);
  STAPL_TEST_REPORT(passed, "Testing view with F3");

  passed = test_matrix_view_coarsening<F4>(n);
  STAPL_TEST_REPORT(passed, "Testing view with F4");

  return EXIT_SUCCESS;
}
