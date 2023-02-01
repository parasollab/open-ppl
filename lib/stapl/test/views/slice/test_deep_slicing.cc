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

struct set_is_deep
{
  using result_type = void;

  template<typename ViewA, typename ViewB>
  void operator()(ViewA vw, ViewB vwb)
  {
    using container_type = typename std::decay<
      typename ViewA::view_container_type
    >::type;

    constexpr bool is_deep = is_deep_slice<container_type>::value;

    for (std::size_t i = 0; i < vw.size(); ++i)
      vw[i] = is_deep;
  }
};

struct slice_is_all_of
{
  using result_type = void;

  template<typename ViewA>
  void operator()(ViewA&& vw)
  {
    auto const first = vw[0];

    bool is_same = true;

    for (std::size_t i = 0; i < vw.size(); ++i)
      if (vw[i] != first)
        is_same = false;

    if (is_same)
      for (std::size_t i = 0; i < vw.size(); ++i)
        vw[i] = 1;
  }
};

struct composed_1d_3d_func
{
  using result_type = void;

  template<typename ViewA>
  void operator()(ViewA vw)
  {
    using container_type = typename std::decay<
      typename ViewA::view_container_type
    >::type;
    using gid_type = typename ViewA::gid_type;

    auto const dom = vw.domain();

    for (std::size_t g = dom.first(); g <= dom.last(); ++g)
    {
      auto v = vw[g];
      auto const d = v.domain();
      constexpr bool is_deep =
        is_deep_slice<typename decltype(v)::view_container_type>::value;

      for (std::size_t i = get<0>(d.first()); i <= get<0>(d.last()); ++i)
        for (std::size_t j = get<1>(d.first()); j <= get<1>(d.last()); ++j)
           for (std::size_t k = get<2>(d.first()); k <= get<2>(d.last()); ++k)
             v(i,j,k) = is_deep;
    }
  }
};

void test_1d_1d(const std::size_t n, const std::size_t m)
{
  using traversal_t     = index_sequence<1, 0>;
  using sliced_dims     = index_sequence<0>;
  using sliced_part_t   = sliced_md_distribution_spec<
    sliced_dims, traversal_t>::type;

  using multiarray_type = multiarray<2, bool, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  auto size = make_tuple(m, n);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(size));
  view_type v(a);

  auto sv1 = make_slices_view<0>(v);

  map_func(set_is_deep(), sv1, sv1);

  auto linear = linear_view(v);
  bool passed = stapl::count(linear, 1) == v.size();

  STAPL_TEST_REPORT(passed, "Subviews are deep slices (1-1)");

  for (std::size_t i = 0; i < m; ++i)
    for (std::size_t j = 0; j < n; ++j)
      v(i, j) = i;
  rmi_fence();

  map_func(slice_is_all_of(), sv1);
  passed = stapl::count(linear, 1) == v.size();

  STAPL_TEST_REPORT(passed, "Deep slices are the right dimensions (1-1)");
}

void test_1d_1d_shallow(const std::size_t n, const std::size_t m)
{
  using traversal_t     = index_sequence<1, 0>;
  using sliced_dims     = index_sequence<1>;
  using sliced_part_t   = sliced_md_distribution_spec<
    sliced_dims, traversal_t>::type;

  using multiarray_type = multiarray<2, bool, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  auto size = make_tuple(m, n);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(size));
  view_type v(a);

  auto sv1 = make_slices_view<1>(v);

  map_func(set_is_deep(), sv1, sv1);

  auto linear = linear_view(v);
  bool passed = stapl::count(linear, 0) == v.size();

  STAPL_TEST_REPORT(passed, "Subviews are shallow slices (1-1)");

  for (std::size_t i = 0; i < m; ++i)
    for (std::size_t j = 0; j < n; ++j)
      v(i, j) = j;
  rmi_fence();

  map_func(slice_is_all_of(), sv1);
  passed = stapl::count(linear, 1) == v.size();

  STAPL_TEST_REPORT(passed, "Shallow slices are the right dimensions (1-1)");
}

void test_2d_1d(const std::size_t n, const std::size_t m)
{
  using traversal_t     = index_sequence<2, 1, 0>;
  using sliced_dims     = index_sequence<0, 1>;
  using sliced_part_t   = sliced_md_distribution_spec<
    sliced_dims, traversal_t>::type;

  using multiarray_type = multiarray<3, std::size_t, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  auto size = make_tuple(m, n, n);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(size));
  view_type v(a);

  auto sv1 = make_slices_view<0, 1>(v);

  map_func(set_is_deep(), sv1, sv1);

  auto linear = linear_view(v);
  bool passed = stapl::count(linear, 1) == v.size();

  STAPL_TEST_REPORT(passed, "Subviews are deep slices (2-1)");

  for (std::size_t i = 0; i < m; ++i)
    for (std::size_t j = 0; j < n; ++j)
      for (std::size_t k = 0; k < n; ++k)
        v(i, j, k) = i*j;

  rmi_fence();

  map_func(slice_is_all_of(), sv1);
  passed = stapl::count(linear, 1) == v.size();

  STAPL_TEST_REPORT(passed, "Deep slices are the right dimensions (2-1)");
}

void test_2d_1d_shallow(const std::size_t n, const std::size_t m)
{
  using traversal_t     = index_sequence<2, 1, 0>;
  using sliced_dims     = index_sequence<1, 2>;
  using sliced_part_t   = sliced_md_distribution_spec<
    sliced_dims, traversal_t>::type;

  using multiarray_type = multiarray<3, std::size_t, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  auto size = make_tuple(m, n, n);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(size));
  view_type v(a);

  auto sv1 = make_slices_view<1, 2>(v);

  map_func(set_is_deep(), sv1, sv1);

  auto linear = linear_view(v);
  bool passed = stapl::count(linear, 0) == v.size();

  STAPL_TEST_REPORT(passed, "Subviews are shallow slices (2-1)");

  for (std::size_t i = 0; i < m; ++i)
    for (std::size_t j = 0; j < n; ++j)
      for (std::size_t k = 0; k < n; ++k)
        v(i, j, k) = j*k;

  rmi_fence();

  map_func(slice_is_all_of(), sv1);
  passed = stapl::count(linear, 1) == v.size();

  STAPL_TEST_REPORT(passed, "Shallow slices are the right dimensions (2-1)");
}

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

  map_func(composed_1d_3d_func(), sv2);

  auto linear = linear_view(v);
  const bool passed = stapl::count(linear, true) == v.size();

  STAPL_TEST_REPORT(passed, "Composed slices + skeletons (1-1-3): deep");
}

exit_code stapl_main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cout << "usage: " << argv[0] << " m n" << std::endl;
    exit(1);
  }

  const std::size_t m = atoi(argv[1]);
  const std::size_t n = atoi(argv[2]);

  test_1d_1d(m,n);
  test_1d_1d_shallow(m,n);
  test_2d_1d(m,n);
  test_2d_1d_shallow(m,n);
  test_1d_1d_3d(m,n);

  return EXIT_SUCCESS;
}
