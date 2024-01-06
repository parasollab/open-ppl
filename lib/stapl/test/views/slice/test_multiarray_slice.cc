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

void test_3d_1d_1d(const std::size_t n, const std::size_t m)
{
  using traversal_t     = index_sequence<4, 3, 2, 1, 0>;
  using sliced_dims     = index_sequence<0, 1, 2>;
  using sliced_part_t   = sliced_md_distribution_spec<
    sliced_dims, traversal_t>::type;
  using value_type = std::array<std::size_t, 5>;

  using multiarray_type = multiarray<5, value_type, sliced_part_t>;

  auto size = make_tuple(m, m, m, m, m);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(size));

  auto& bc = *(a.distribution().container_manager().begin());

  using base_container_type = std::decay<decltype(bc)>::type;
  using view_type       = multiarray_view<base_container_type>;

  view_type v(bc);

  for (std::size_t i = 0; i < m; ++i)
    for (std::size_t j = 0; j < m; ++j)
      for (std::size_t k = 0; k < m; ++k)
        for (std::size_t d = 0; d < m; ++d)
          for (std::size_t g = 0; g < m; ++g)
            v(i,j,k,d,g) = value_type{{i,j,k,d,g}};

  auto sv1 = make_slices_view<0, 1, 2, 3>(v);
  auto sv2 = make_slices_view<0, 1, 2>(sv1);

  bool passed = true;
  for (std::size_t i = 0; i < m; ++i)
  {
    for (std::size_t j = 0; j < m; ++j)
    {
      for (std::size_t k = 0; k < m; ++k)
      {
        auto w = sv2(i,j,k);

        for (std::size_t g = 0; g < m; ++g)
        {
          auto x = w[g];

          constexpr bool is_deep =
            is_deep_slice<typename decltype(x)::view_container_type>::value;

          for (std::size_t d = 0; d < m; ++d)
          {
            value_type val = x[d];
            value_type expected{{i,j,k,g,d}};
            const bool is_correct_index =
              std::equal(val.begin(), val.end(), expected.begin());
            if (!is_deep || !is_correct_index)
              passed = false;
          }
        }
      }
    }
  }

  STAPL_TEST_REPORT(passed, "Testing 3-1-1");
}



void test_1d_1d_3d(const std::size_t n, const std::size_t m)
{
  using traversal_t     = index_sequence<4, 3, 2, 1, 0>;
  using sliced_dims     = index_sequence<0>;
  using sliced_part_t   = sliced_md_distribution_spec<
    sliced_dims, traversal_t>::type;

  using multiarray_type = multiarray<5, bool, sliced_part_t>;

  auto size = make_tuple(m, m, m, n, n);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(size));

  auto& bc = *(a.distribution().container_manager().begin());

  using base_container_type = std::decay<decltype(bc)>::type;
  using view_type       = multiarray_view<base_container_type>;

  view_type v(bc);

  auto sv1 = make_slices_view<0, 1>(v);
  auto sv2 = make_slices_view<0>(sv1);

  for (std::size_t g = 0; g < m; ++g)
  {
    auto w = sv2[g];
    for (std::size_t d = 0; d < m; ++d)
    {
      auto x = w[d];

      constexpr bool is_deep =
        is_deep_slice<typename decltype(x)::view_container_type>::value;

      for (std::size_t i = 0; i < m; ++i)
        for (std::size_t j = 0; j < n; ++j)
          for (std::size_t k = 0; k < n; ++k)
            x(i,j,k) = is_deep;
    }
  }

  // check
  bool passed = true;
  for (std::size_t g = 0; g < m; ++g)
    for (std::size_t d = 0; d < m; ++d)
      for (std::size_t i = 0; i < m; ++i)
        for (std::size_t j = 0; j < n; ++j)
          for (std::size_t k = 0; k < n; ++k)
            if (!bc(d,g,i,j,k))
              passed = false;

  STAPL_TEST_REPORT(passed, "Testing 1-1-3");
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

  test_1d_1d_3d(n, m);
  test_3d_1d_1d(n, m);

  return EXIT_SUCCESS;
}
