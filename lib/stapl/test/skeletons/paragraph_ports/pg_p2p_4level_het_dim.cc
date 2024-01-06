/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
#include <tuple>
#include <vector>
#include <utility>

#include <stapl/containers/array/array.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/skeletons/functional/wavefront.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/spans/blocked.hpp>
#include <stapl/skeletons/transformations/transform.hpp>
#include <stapl/skeletons/transformations/nest.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple.hpp>

#include "../../expect.hpp"
#include "nested_utils.hpp"
#include "nested_multiarray.hpp"

using namespace stapl;
using namespace skeletons;
using namespace wavefront_utils;

using timer_type = counter<default_timer>;

constexpr size_t ndims0   = 2;
constexpr size_t ndims1   = 2;
constexpr size_t ndims2   = 2;
constexpr size_t ndims3   = 3;
constexpr size_t nlevels  = 4;

using value_type   = int;
using dims_type0   = typename homogeneous_tuple_type<ndims0, size_t>::type;
using dims_type1   = typename homogeneous_tuple_type<ndims1, size_t>::type;
using dims_type2   = typename homogeneous_tuple_type<ndims2, size_t>::type;
using dims_type3   = typename homogeneous_tuple_type<ndims3, size_t>::type;
using dims_tuple_t = tuple<dims_type0, dims_type1, dims_type2, dims_type3>;

template<typename ViewTuple, typename Corner, std::size_t... index>
void test_wf_zip_wf_zip(ViewTuple&& in_view, Corner&& start,
                        stapl::index_sequence<index...>)
{
  auto zip_sk = zip<ndims0 + 1>(
                  sum_op<ndims0, value_type>(),
                  skeleton_traits<spans::blocked<ndims3>, true>());

  auto wf_zip = wavefront(zip_sk, generate_corner<ndims2>(start),
                          skeleton_traits<spans::blocked<ndims2>, true>());

  auto zip_wf_zip = zip<ndims0 + 1>(
                      wf_zip, skeleton_traits<spans::blocked<ndims1>, true>());

  auto wf_zip_wf_zip = wavefront(zip_wf_zip, generate_corner<ndims0>(start),
                                 skeleton_traits<spans::blocked<ndims0>>());
  auto nested_wf_zip_wf_zip = nest(wf_zip_wf_zip);

  std::string title =
    std::to_string(ndims0) + "D wavefront(" +
    std::to_string(ndims1) + "D zip("       +
    std::to_string(ndims2) + "D wavefront(" +
    std::to_string(ndims3) + "D zip(+)))) non-coarsened";

  timer_type timer;
  timer.start();

  skeletons::execute(
    skeletons::execution_params(),
    nested_wf_zip_wf_zip,
    std::get<index>(std::forward<ViewTuple>(in_view))...);

  double elapsed_time = timer.stop();
  print_timer_values(title, elapsed_time);

#ifdef VALIDATE_RESULTS
  stapl::do_once([&](void) {
    stapl::tests::expect(verify<nlevels>(std::get<0>(in_view))) << title;
  });
#endif

  return;
}

template<typename ViewTuple, typename Corner, std::size_t... index>
void test_wf_wf_zip_zip(ViewTuple&& in_view, Corner&& start,
                        stapl::index_sequence<index...>)
{
  auto zip_sk =
    zip<ndims0 + 1>(sum_op<ndims0, value_type>(),
                    skeleton_traits<spans::blocked<ndims3>, true>());

  auto zip_zip =
    zip<ndims0 + 1>(zip_sk, skeleton_traits<spans::blocked<ndims2>, true>());

  auto wf_zip_zip =
    wavefront(zip_zip,
              generate_corner<ndims1>(start),
              skeleton_traits<spans::blocked<ndims1>, true>());

  auto wf_wf_zip_zip =
    wavefront(wf_zip_zip,
              generate_corner<ndims0>(start),
              skeleton_traits<spans::blocked<ndims0>>());

  auto nested_wf_wf_zip_zip = nest(wf_wf_zip_zip);

  std::string title =
    std::to_string(ndims0) + "D wavefront(" +
    std::to_string(ndims1) + "D wavefront(" +
    std::to_string(ndims2) + "D zip("       +
    std::to_string(ndims3) + "D zip(+)))) non-coarsened";

  timer_type timer;
  timer.start();

  skeletons::execute(
    skeletons::execution_params(),
    nested_wf_wf_zip_zip,
    std::get<index>(std::forward<ViewTuple>(in_view))...);

  double elapsed_time = timer.stop();
  print_timer_values(title, elapsed_time);

#ifdef VALIDATE_RESULTS
  stapl::do_once([&](void) {
    stapl::tests::expect(verify<nlevels>(std::get<0>(in_view))) << title;
  });
#endif

  return;
}

void test_4_levels(
       size_t n_level0, size_t n_level1, size_t n_level2, size_t n_level3)
{
  dims_type0 dims_level0 = homogeneous_tuple<ndims0>(n_level0);
  dims_type1 dims_level1 = homogeneous_tuple<ndims1>(n_level1);
  dims_type2 dims_level2 = homogeneous_tuple<ndims2>(n_level2);
  dims_type3 dims_level3 = homogeneous_tuple<ndims3>(n_level3);

  using nested_multiarray_t =
    typename nested_multiarray<nlevels - 1, dims_tuple_t, value_type>::type;

  dims_tuple_t dims_tuple =
    make_tuple(dims_level0, dims_level1, dims_level2, dims_level3);

  auto&& dims_vw1 =
    make_nested_dims_container<nlevels - 1, dims_tuple_t>::create(dims_tuple);

  nested_multiarray_t in_container_4level(dims_vw1);
  multiarray_view<nested_multiarray_t> in_view_4level(in_container_4level);

  auto&& first_idx0 = tuple_ops::extract_1D(homogeneous_tuple<ndims0>(0));
  auto&& first_idx1 = tuple_ops::extract_1D(homogeneous_tuple<ndims1>(0));
  auto&& first_idx2 = tuple_ops::extract_1D(homogeneous_tuple<ndims2>(0));
  auto&& first_idx3 = tuple_ops::extract_1D(homogeneous_tuple<ndims3>(0));

  in_view_4level[first_idx0][first_idx1][first_idx2][first_idx3] = 1;

  auto f = position::first;

  auto&& inputs_tuple = homogeneous_tuple<4>(in_view_4level);

  test_wf_zip_wf_zip(inputs_tuple, f, stapl::make_index_sequence<ndims0+1>());
  test_wf_wf_zip_zip(inputs_tuple, f, stapl::make_index_sequence<ndims0+1>());
  return;
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 5 ){
    std::cout << "<exec> " << std::endl;
    exit(1);
  }

  const int num_elems_level0 = atoi(argv[1]);
  const int num_elems_level1 = atoi(argv[2]);
  const int num_elems_level2 = atoi(argv[3]);
  const int num_elems_level3 = atoi(argv[4]);

  test_4_levels(
    num_elems_level0, num_elems_level1, num_elems_level2, num_elems_level3);

  return EXIT_SUCCESS;
}
