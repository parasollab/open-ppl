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
constexpr size_t nlevels  = 2 ;

using value_type   = int;
using dims_type0   = typename homogeneous_tuple_type<ndims0, size_t>::type;
using dims_type1   = typename homogeneous_tuple_type<ndims1, size_t>::type;
using dims_tuple_t = tuple<dims_type0, dims_type1>;

template <int dim0, int dim1, typename ViewTuple, typename Corner,
          std::size_t... index>
void test_wf_of_zip(ViewTuple&& in_view, Corner&& start,
                    index_sequence<index...>)
{
  auto zip_sk = zip<dim0 + 1>(sum_op<dim0, value_type>(),
                             skeletons::skeleton_traits<
                               spans::blocked<dim1>, true>());
  auto wf_of_zip = wavefront(zip_sk, generate_corner<dim0>(start));

  auto nested_wf_zip = nest(wf_of_zip);

  std::string title = std::to_string(dim0) + "D wavefront(zip) non-coarsened";
  timer_type timer;
  timer.start();

  skeletons::execute(
    skeletons::execution_params(),
    nested_wf_zip, std::get<index>(std::forward<ViewTuple>(in_view))...);

  double elapsed_time = timer.stop();
  print_timer_values(title, elapsed_time);
#ifdef VALIDATE_RESULTS
  stapl::do_once([&](void) {
    stapl::tests::expect(verify<nlevels>(std::get<0>(in_view))) << title;
  });
#endif
  return;
}

void test_2_levels(size_t n_level0, size_t n_level1)
{
  dims_type0 dims_level0 = homogeneous_tuple<ndims0>(n_level0);
  dims_type1 dims_level1 = homogeneous_tuple<ndims1>(n_level1);

  using nested_multiarray_t =
    typename nested_multiarray<nlevels - 1, dims_tuple_t, value_type>::type;

  dims_tuple_t dims_tuple = make_tuple(dims_level0, dims_level1);

  auto&& dims_vw1 =
    make_nested_dims_container<nlevels - 1, dims_tuple_t>::create(dims_tuple);

  nested_multiarray_t in_container_2level(dims_vw1);
  multiarray_view<nested_multiarray_t> in_view_2level(in_container_2level);

  auto&& first_idx0 = homogeneous_tuple<ndims0>(0);
  auto&& first_idx1 = homogeneous_tuple<ndims1>(0);

  in_view_2level[first_idx0][first_idx1] = 1;

  using corner_t = skeletons::position;
  auto f = position::first;

  auto&& inputs_tuple = homogeneous_tuple<ndims0 + 1>(in_view_2level);

  test_wf_of_zip<ndims0, ndims1>(inputs_tuple, f,
                                 make_index_sequence<ndims0 + 1>());
}

exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3 ){
    std::cout << "<exec> " << std::endl;
    exit(1);
  }

  const int num_elems_level0 = atoi(argv[1]);
  const int num_elems_level1 = atoi(argv[2]);

  test_2_levels(num_elems_level0, num_elems_level1);

  return EXIT_SUCCESS;
}
