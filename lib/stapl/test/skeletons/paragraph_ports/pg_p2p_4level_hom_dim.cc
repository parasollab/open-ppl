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
constexpr size_t ndims   = CUR_DIM;
constexpr size_t nlevels = 4 ;

using value_type   = int;
using dims_type    = homogeneous_tuple_type_t<ndims, size_t>;
using dims_tuple_t = homogeneous_tuple_type_t<nlevels, dims_type>;

template<int dim, typename ViewTuple, typename Corner, std::size_t... index>
void test_wf_wf_wf_wf(ViewTuple&& in_view, Corner&& start,
                      stapl::index_sequence<index...>)
{
  auto&& traits = skeleton_traits<stapl::use_default, true>();
  auto&& corner = generate_corner<ndims>(start);

  auto wf_sk       = wavefront(sum_op<dim, value_type>(), corner, traits);
  auto wf_wf       = wavefront(wf_sk, corner, traits);
  auto wf_wf_wf    = wavefront(wf_wf, corner, traits);
  auto wf_wf_wf_wf = wavefront(wf_wf_wf, corner);

  auto nested_wf_wf_wf_wf  = nest(wf_wf_wf_wf);

  std::string title =
    std::to_string(ndims) + "Dwavefront(" +
    std::to_string(ndims) + "Dwavefront(" +
    std::to_string(ndims) + "Dwavefront(" +
    std::to_string(ndims) + "Dwavefront(+)))) non-coarsened";

  timer_type timer;
  timer.start();

  skeletons::execute(
    skeletons::execution_params(),
    nested_wf_wf_wf_wf,
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

template<int dim, typename ViewTuple, typename Corner, std::size_t... index>
void test_wf_zip_wf_zip(ViewTuple&& in_view, Corner&& start,
                        stapl::index_sequence<index...>)
{
  auto&& traits = skeleton_traits<stapl::use_default, true>();
  auto&& corner = generate_corner<ndims>(start);

  auto zip_sk        = zip<dim + 1>(sum_op<dim, value_type>(),
                                    skeletons::skeleton_traits<
                                      spans::blocked<dim>, true>());
  auto wf_zip        = wavefront(zip_sk, corner, traits);
  auto zip_wf_zip    = zip<dim + 1>(wf_zip,
                                    skeletons::skeleton_traits<
                                      spans::blocked<dim>, true>());
  auto wf_zip_wf_zip = wavefront(zip_wf_zip, corner);
  auto nested_wf_zip_wf_zip = nest(wf_zip_wf_zip);

  std::string title =
    std::to_string(ndims) + "Dwavefront(" +
    std::to_string(ndims) + "Dzip(" +
    std::to_string(ndims) + "Dwavefront(" +
    std::to_string(ndims) + "Dzip(+)))) non-coarsened";

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

void test_4_levels(
       size_t n_level0, size_t n_level1, size_t n_level2, size_t n_level3)
{
  dims_type dims_level0 = homogeneous_tuple<ndims>(n_level0);
  dims_type dims_level1 = homogeneous_tuple<ndims>(n_level1);
  dims_type dims_level2 = homogeneous_tuple<ndims>(n_level2);
  dims_type dims_level3 = homogeneous_tuple<ndims>(n_level3);

  using nested_multiarray_t =
    typename nested_multiarray<nlevels - 1, dims_tuple_t, value_type>::type;

  dims_tuple_t dims_tuple =
    make_tuple(dims_level0, dims_level1, dims_level2, dims_level3);

  auto&& dims_vw1 =
    make_nested_dims_container<nlevels - 1, dims_tuple_t>::create(dims_tuple);

  nested_multiarray_t in_container_3level(dims_vw1);
  multiarray_view<nested_multiarray_t> in_view_3level(in_container_3level);

  auto&& first_idx = homogeneous_tuple<ndims>(0);
  in_view_3level[first_idx][first_idx][first_idx][first_idx] = 1;

  auto f = position::first;

  auto&& inputs_tuple = homogeneous_tuple<ndims + 1>(in_view_3level);

  test_wf_wf_wf_wf<ndims>(inputs_tuple, f,
                          stapl::make_index_sequence<ndims + 1>());
  test_wf_zip_wf_zip<ndims>(inputs_tuple, f,
                            stapl::make_index_sequence<ndims + 1>());

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
