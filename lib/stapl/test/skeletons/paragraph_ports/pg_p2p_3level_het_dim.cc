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
#include <stapl/skeletons/transformations/transform.hpp>
#include <stapl/skeletons/transformations/nest.hpp>
#include <stapl/skeletons/spans/blocked.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple.hpp>

#include "../../expect.hpp"
#include "nested_utils.hpp"
#include "nested_multiarray.hpp"

using namespace skeletons;
using namespace wavefront_utils;

using timer_type = counter<default_timer>;

constexpr size_t ndims0   = 3;
constexpr size_t ndims1   = 2;
constexpr size_t ndims2   = 3;
constexpr size_t nlevels  = 3;

using value_type   = int;
using dims_type0   = typename homogeneous_tuple_type<ndims0, size_t>::type;
using dims_type1   = typename homogeneous_tuple_type<ndims1, size_t>::type;
using dims_type2   = typename homogeneous_tuple_type<ndims2, size_t>::type;
using dims_tuple_t = tuple<dims_type0, dims_type1, dims_type2>;

template <typename ViewTuple, typename Corner, std::size_t... index>
void test_wf_of_zip_of_zip(ViewTuple&& in_view, Corner&& start,
                           stapl::index_sequence<index...>)
{
  auto zip_sk         = zip<ndims0 + 1>(sum_op<ndims0, value_type>(),
                                      skeletons::skeleton_traits<
                                        spans::blocked<ndims2>, true>());
  auto zip_zip        = zip<ndims0 + 1>(zip_sk,
                                      skeletons::skeleton_traits<
                                        spans::blocked<ndims1>, true>());
  auto wf_zip_wf      = wavefront(zip_zip, generate_corner<ndims0>(start));
  auto nest_wf_zip_wf = nest(wf_zip_wf);

  std::string title =
    std::to_string(ndims0) + "D wavefront(" +
    std::to_string(ndims1) + "D zip(" +
    std::to_string(ndims2) + "D zip(+))) non-coarsened";
  timer_type timer;
  timer.start();

  skeletons::execute(
    skeletons::execution_params(),
    nest_wf_zip_wf,
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

template <typename ViewTuple, typename Corner, std::size_t... index>
void test_zip_of_wf_of_zip(ViewTuple&& in_view, Corner&& start,
                           stapl::index_sequence<index...>)
{
  auto&& traits  = skeleton_traits<spans::blocked<ndims1>, true>();

  auto zip_sk     = zip<ndims1 + 1>(sum_op<ndims1, value_type>(),
                                    skeletons::skeleton_traits<
                                      spans::blocked<ndims2>, true>());
  auto wf_zip     = wavefront(zip_sk, generate_corner<ndims1>(start), traits);
  auto zip_wf_zip =
                    zip<ndims1 + 1>(
                      wf_zip, skeleton_traits<spans::blocked<ndims0>>());
  auto nested_zip_wf_zip = nest(zip_wf_zip);

  std::string title =
    std::to_string(ndims0) + "D zip(" +
    std::to_string(ndims1) + "D wavefront(" +
    std::to_string(ndims2) + "D zip(+))) non-coarsened";
  timer_type timer;
  timer.start();

  skeletons::execute(
    skeletons::execution_params(),
    nested_zip_wf_zip, std::get<index>(std::forward<ViewTuple>(in_view))...);

  double elapsed_time = timer.stop();
  print_timer_values(title, elapsed_time);
#ifdef VALIDATE_RESULTS
  stapl::do_once([&](void) {
    stapl::tests::expect(verify<nlevels>(std::get<0>(in_view))) << title;
  });
#endif
  return;
}


template <typename ViewTuple, typename Corner,
          std::size_t... index>
void test_zip_of_zip_of_wf(ViewTuple&& in_view, Corner&& start,
                           stapl::index_sequence<index...>)
{

  auto&& traits = skeleton_traits<spans::blocked<ndims2>, true>();

  auto wf_sk = wavefront(
    sum_op<ndims2, value_type>(), generate_corner<ndims2>(start), traits);

  auto zip_wf = zip<ndims2 + 1>(
    wf_sk, skeletons::skeleton_traits<spans::blocked<ndims1>, true>());

  auto zip_zip_wf = zip<ndims2 + 1>(
    zip_wf, skeletons::skeleton_traits<spans::blocked<ndims0>>());

  auto nest_zip_zip_wf = nest(zip_zip_wf);

  std::string title =
    std::to_string(ndims0) + "D zip(" +
    std::to_string(ndims1) + "D zip(" +
    std::to_string(ndims2) + "D wavefront(+))) non-coarsened";
  timer_type timer;
  timer.start();

  skeletons::execute(
    skeletons::execution_params(),
    nest_zip_zip_wf,
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

void test_3_levels(size_t n_level0, size_t n_level1, size_t n_level2)
{
  dims_type0 dims_level0 = homogeneous_tuple<ndims0>(n_level0);
  dims_type1 dims_level1 = homogeneous_tuple<ndims1>(n_level1);
  dims_type2 dims_level2 = homogeneous_tuple<ndims2>(n_level2);

  using nested_multiarray_t =
    typename nested_multiarray<nlevels-1, dims_tuple_t, value_type>::type;

  dims_tuple_t dims_tuple = make_tuple(dims_level0, dims_level1, dims_level2);

  auto&& dims_vw1 =
    make_nested_dims_container<nlevels-1, dims_tuple_t>::create(dims_tuple);

  nested_multiarray_t in_container_3level(dims_vw1);
  multiarray_view<nested_multiarray_t> in_view_3level(in_container_3level);

  auto&& first_idx0 = homogeneous_tuple<ndims0>(0);
  auto&& first_idx1 = homogeneous_tuple<ndims1>(0);
  auto&& first_idx2 = homogeneous_tuple<ndims2>(0);
  in_view_3level[first_idx0][first_idx1][first_idx2] = 1;

  auto f = position::first;

  auto&& inputs_tuple = homogeneous_tuple<4>(in_view_3level);

  test_wf_of_zip_of_zip(inputs_tuple, f, make_index_sequence<ndims0+1>());
  test_zip_of_wf_of_zip(inputs_tuple, f, make_index_sequence<ndims1+1>());
  test_zip_of_zip_of_wf(inputs_tuple, f, make_index_sequence<ndims2+1>());
}

exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3 ){
    std::cout << "<exec> " << std::endl;
    exit(1);
  }

  const int num_elems_level0 = atoi(argv[1]);
  const int num_elems_level1 = atoi(argv[2]);
  const int num_elems_level2 = atoi(argv[3]);

  test_3_levels(num_elems_level0, num_elems_level1, num_elems_level2);

  return EXIT_SUCCESS;
}
