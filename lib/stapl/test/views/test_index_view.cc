/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/multiarray.hpp>
#include <stapl/skeletons/spans/blocked.hpp>
#include <stapl/views/slices_view.hpp>
#include <stapl/views/index_view.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../test_report.hpp"

using namespace stapl;

using value_t = std::tuple<size_t, size_t, size_t, size_t, size_t>;


struct dgz_wf
{
  template<typename ViewA, typename ViewB>
  void operator()(ViewA vwa, ViewB vwb)
  {
    for (size_t i=0; i != vwa.size(); ++i)
    {
      auto vw2   = vwa[i];
      auto sizes = vw2.dimensions();
      for (size_t j = 0; j != get<0>(sizes); ++j)
        for (size_t k = 0; k != get<1>(sizes); ++k)
          for (size_t l = 0; l != get<2>(sizes); ++l)
            vw2(j,k,l) = vwb[i](j,k,l);
    }
  }
};


struct gzd_wf
{
  template<typename ViewA, typename ViewB>
  void operator()(ViewA vwa, ViewB vwb)
  {
    auto sizes = vwa.dimensions();
    for (size_t j = 0; j != get<0>(sizes); ++j)
      for (size_t k = 0; k != get<1>(sizes); ++k)
        for (size_t l = 0; l != get<2>(sizes); ++l)
        {
          auto vw2 = vwa(j, k, l);
          for (size_t i=0; i != vw2.size(); ++i)
          vw2[i] = vwb(j,k,l)[i];
        }
  }
};


struct zgd_wf
{
  template<typename ViewA, typename ViewB>
  void operator()(ViewA vwa, ViewB vwb)
  {
    for (size_t i=0; i != vwa.size(); ++i)
    {
      auto vw2 = vwa[i];
      for (size_t j = 0; j != vw2.size(); ++j)
        vw2[j] = vwb[i][j];
    }
  }
};


void test_dgz(void)
{
  using traversal_t     = index_sequence<2, 1, 0, 4, 3>;
  using sliced_dims     = index_sequence<3>;
  using sliced_part_t   =
    sliced_md_distribution_spec<sliced_dims, traversal_t>::type;
  using multiarray_type = multiarray<5, value_t, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  auto sizea = make_tuple(4, 5, 6, 7, 8);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(sizea));
  view_type va(a);

  auto sva1 = make_slices_view<3, 4>(va);
  auto sva2 = make_slices_view<0>(sva1);

  auto vb   = index_view(a);
  auto svb1 = make_slices_view<3, 4>(vb);
  auto svb2 = make_slices_view<0>(svb1);

  skeletons::execute(
    skeletons::execution_params(multiview_coarsener<true>()),
    skeletons::coarse(skeletons::zip<2>(dgz_wf())), sva2, svb2);

  bool passed = true;

  do_once([&]() {
    for (size_t i = 0; i < get<0>(a.dimensions()); ++i)
      for (size_t j = 0; j < get<1>(a.dimensions()); ++j)
        for (size_t k = 0; k < get<2>(a.dimensions()); ++k)
          for (size_t d = 0; d < get<3>(a.dimensions()); ++d)
            for (size_t g = 0; g < get<4>(a.dimensions()); ++g)
              if (make_tuple(i,j,k,d,g) != a(i,j,k,d,g))
                passed = false;
  });

  STAPL_TEST_REPORT(passed, "Test DGZ slice index_view");
}


void test_gzd(void)
{
  using traversal_t     = index_sequence<3, 2, 1, 0, 4>;
  using sliced_dims     = index_sequence<4>;
  using sliced_part_t   =
    sliced_md_distribution_spec<sliced_dims, traversal_t>::type;
  using multiarray_type = multiarray<5, value_t, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  auto sizea = make_tuple(4, 5, 6, 7, 8);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(sizea));
  view_type va(a);

  auto sva1 = make_slices_view<0, 1, 2, 4>(va);
  auto sva2 = make_slices_view<3>(sva1);

  auto vb   = index_view(a);
  auto svb1 = make_slices_view<0, 1, 2, 4>(vb);
  auto svb2 = make_slices_view<3>(svb1);

  skeletons::execute(
    skeletons::execution_params(multiview_coarsener<true>()),
    skeletons::coarse(skeletons::zip<2>(gzd_wf())), sva2, svb2);

  bool passed = true;

  do_once([&]() {
    for (size_t i = 0; i < get<0>(a.dimensions()); ++i)
      for (size_t j = 0; j < get<1>(a.dimensions()); ++j)
        for (size_t k = 0; k < get<2>(a.dimensions()); ++k)
          for (size_t d = 0; d < get<3>(a.dimensions()); ++d)
            for (size_t g = 0; g < get<4>(a.dimensions()); ++g)
              if (make_tuple(i,j,k,d,g) != a(i,j,k,d,g))
                passed = false;
  });

  STAPL_TEST_REPORT(passed, "Test GZD slice index_view");
}


void test_zgd(void)
{
  using traversal_t     = index_sequence<4, 3, 2, 0, 1>;
  using sliced_dims     = index_sequence<0, 1, 2>;
  using sliced_part_t   =
    sliced_md_distribution_spec<sliced_dims, traversal_t>::type;
  using multiarray_type = multiarray<5, value_t, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  auto sizea = make_tuple(4, 5, 6, 7, 8);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(sizea));
  view_type va(a);

  auto sva1 = make_slices_view<0, 1, 2, 4>(va);
  auto sva2 = make_slices_view<0, 1, 2>(sva1);

  auto vb   = index_view(a);
  auto svb1 = make_slices_view<0, 1, 2, 4>(vb);
  auto svb2 = make_slices_view<0, 1, 2>(svb1);

  using span_t = skeletons::spans::blocked<3>;

  skeletons::execute(
   skeletons::execution_params(multiview_coarsener<true>()),
    skeletons::coarse(
      skeletons::zip<2>(zgd_wf(), skeletons::skeleton_traits<span_t>())),
    sva2, svb2);

  bool passed = true;

  do_once([&]() {
    for (size_t i = 0; i < get<0>(a.dimensions()); ++i)
      for (size_t j = 0; j < get<1>(a.dimensions()); ++j)
        for (size_t k = 0; k < get<2>(a.dimensions()); ++k)
          for (size_t d = 0; d < get<3>(a.dimensions()); ++d)
            for (size_t g = 0; g < get<4>(a.dimensions()); ++g)
              if (make_tuple(i,j,k,d,g) != a(i,j,k,d,g))
                passed = false;
  });

  STAPL_TEST_REPORT(passed, "Test ZGD slice index_view");
}

exit_code stapl_main(int argc, char** argv)
{
  test_dgz();
  test_gzd();
  test_zgd();

  return EXIT_SUCCESS;
}
