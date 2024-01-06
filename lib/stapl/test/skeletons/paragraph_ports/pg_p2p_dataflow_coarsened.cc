/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <cstdlib>

#include <stapl/runtime.hpp>
#include <stapl/array.hpp>
#include <stapl/skeletons/functional/map.hpp>
#include <stapl/skeletons/functional/map_reduce.hpp>
#include <stapl/skeletons/functional/broadcast_to_locs.hpp>
#include <stapl/skeletons/transformations/transform.hpp>
#include <stapl/skeletons/transformations/nest.hpp>
#include <stapl/skeletons/transformations/coarse.hpp>
#include <stapl/skeletons/flows/inline_flows.hpp>
#include <stapl/views/metadata/coarseners/pg_aware_multiview_coarsener.hpp>
#include "nested_utils.hpp"
#include "../../expect.hpp"

using namespace stapl;
using namespace skeletons;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3){
    std::cout << "<exec> <n>" << std::endl;
    exit(1);
  }

  size_t n = atoi(argv[1]);
  size_t m = atoi(argv[2]);

  using value_t = int;
  using array_type = array<array<value_t>>;

  // Inner container distributed across the system
  array<value_t>                  c0(m);
  array_type                      c(n,c0);
  array_view<array_type>          x_vw(c);

  execute(default_execution_params(),
          coarse(zip<1>(stapl::bind1st(assign<value_t>(), 1.0))),
          x_vw);

  sum_op<1, value_t> plus_wf;

  auto&& inner_sk_traits =
    skeleton_traits<spans::blocked<1>, true>(
      get_filters<1>(), get_nested_mappers<1>());

  auto&& zip_sk = coarse(
    zip<2>(plus_wf, inner_sk_traits));

  auto&& outer_sk_traits =
    skeleton_traits<spans::blocked<1>, false>(
      get_filters<1>(), get_nested_mappers<1>());

  auto&& row_id =
    zip<2>(zip_sk, outer_sk_traits,
           execution_params(pg_aware_multiview_coarsener()));


  size_t num_locs = get_num_locations();
  std::array<size_t, 2> level_dimensions = {{n, std::max(num_locs, m)}};

  row_id.span().set_level_dims(level_dimensions);

  namespace ph = stapl::skeletons::flows::inline_flows::placeholders;

  ph::input<0> x_in;
  ph::input<1> y_in;

  DECLARE_INLINE_PLACEHOLDERS(2, x);

  auto&& transpose_sk =
  compose<skeletons::tags::inline_flow>(
    x0 << row_id | (x_in, y_in),
    x1 << row_id | (y_in, x0)
  );

  using timer_type = counter<default_timer>;
  timer_type timer;
  timer.start();

  execute(execution_params(null_coarsener()),
          transpose_sk,
          x_vw, x_vw);

  double elapsed_time = timer.stop();

  auto&& s1 = compose(
                skeletons::map_reduce(
                  reduce(stapl::plus<value_t>()), stapl::plus<value_t>()),
                broadcast_to_locs<true>()
              );

  size_t r = skeletons::execute(
                    skeletons::execution_params<value_t>(), s1, x_vw);

  std::string title = "Testing compose(zip(coarse(zip)), zip(coarse(zip))))";

  stapl::do_once([&](void) {
    print_timer_values(title, elapsed_time);
    stapl::tests::expect(4 * (m * n) == r) << title;
  });

  return EXIT_SUCCESS;
}
