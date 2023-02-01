/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/multiarray.hpp>
#include <stapl/views/slices_view.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include "../../test_report.hpp"

using namespace stapl;

struct set_is_local
{
  using result_type = void;

  template<typename ViewA, typename ViewB>
  void operator()(ViewA vw, ViewB vwb)
  {
    domain_map(vw.domain(), [&](typename ViewA::index_type const& i) {
      auto sv = vw[i];
      auto dom = sv.domain();

      domain_map(dom, [&](decltype(dom.first()) const& j) {

        auto x = sv[j];

        const bool is_local =
          accessor_core_access::is_local(proxy_core_access::accessor(x));

        x = is_local;
      });
    });
  }
};

void test_1d_1d_3d(const std::size_t n, const std::size_t m)
{
  using traversal_t     = index_sequence<4, 3, 2, 1, 0>;
  using sliced_dims     = index_sequence<0>;
  using sliced_part_t   = sliced_md_distribution_spec<
    sliced_dims, traversal_t>::type;

  using multiarray_type = multiarray<5, bool, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  auto size = make_tuple(m, m, m, n, n);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(size));
  view_type v(a);

  auto sv1 = make_slices_view<0, 1>(v);
  auto sv2 = make_slices_view<0>(sv1);

  map_func(set_is_local(), sv2, sv2);

  auto linear = linear_view(v);
  const bool passed = stapl::count(linear, true) == v.size();

  STAPL_TEST_REPORT(passed, "Composed slices + skeletons (1-1-3): deep");
}

void test_1d_1d_3d_shallow(const std::size_t n, const std::size_t m)
{
  using traversal_t     = index_sequence<4, 3, 2, 1, 0>;
  using sliced_dims     = index_sequence<1>;
  using sliced_part_t   = sliced_md_distribution_spec<
    sliced_dims, traversal_t>::type;

  using multiarray_type = multiarray<5, bool, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  auto size = make_tuple(m, m, m, n, n);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(size));
  view_type v(a);

  auto sv1 = make_slices_view<0, 1>(v);
  auto sv2 = make_slices_view<1>(sv1);

  map_func(set_is_local(), sv2, sv2);

  auto linear = linear_view(v);
  const bool passed = stapl::count(linear, true) == v.size();

  STAPL_TEST_REPORT(passed, "Composed slices + skeletons (1-1-3): shallow");
}

void test_3d_1d_1d(const std::size_t n, const std::size_t m)
{
  using traversal_t     = index_sequence<4, 3, 2, 1, 0>;
  using sliced_dims     = index_sequence<0, 1, 2>;
  using sliced_part_t   = sliced_md_distribution_spec<
    sliced_dims, traversal_t>::type;

  using multiarray_type = multiarray<5, bool, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  auto size = make_tuple(m, m, m, n, n);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(size));
  view_type v(a);

  auto sv1 = make_slices_view<0, 1, 2, 3>(v);
  auto sv2 = make_slices_view<0, 1, 2>(sv1);

  map_func(set_is_local(), sv2, sv2);

  auto linear = linear_view(v);
  const bool passed = stapl::count(linear, true) == v.size();

  STAPL_TEST_REPORT(passed, "Composed slices + skeletons (3-1-1): deep");
}

void test_3d_1d_1d_shallow(const std::size_t n, const std::size_t m)
{
  using traversal_t     = index_sequence<4, 0, 1, 2, 3>;
  using sliced_dims     = index_sequence<0, 1, 2>;
  using sliced_part_t   = sliced_md_distribution_spec<
    sliced_dims, traversal_t>::type;

  using multiarray_type = multiarray<5, bool, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  auto size = make_tuple(m, m, m, n, n);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(size));
  view_type v(a);

  auto sv1 = make_slices_view<0, 1, 2, 3>(v);
  auto sv2 = make_slices_view<0, 1, 2>(sv1);

  map_func(set_is_local(), sv2, sv2);

  auto linear = linear_view(v);
  const bool passed = stapl::count(linear, true) == v.size();

  STAPL_TEST_REPORT(passed, "Composed slices + skeletons (3-1-1): shallow");
}

exit_code stapl_main(int argc, char** argv)
{
  const std::size_t m = atoi(argv[1]);
  const std::size_t n = atoi(argv[2]);

  test_3d_1d_1d(m,n);
  test_3d_1d_1d_shallow(m,n);
  test_1d_1d_3d(m,n);
  test_1d_1d_3d_shallow(m,n);

  return EXIT_SUCCESS;
}
