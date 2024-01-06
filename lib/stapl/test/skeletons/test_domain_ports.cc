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
#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/functional/map.hpp>
#include <stapl/skeletons/functional/reduce.hpp>
#include <stapl/skeletons/flows/view_flow.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/operators/do_while.hpp>
#include <stapl/skeletons/functional/scan.hpp>

#include <stapl/utility/tuple.hpp>

#include "../expect.hpp"

using namespace stapl;

template<typename Sk, typename InFlow>
void test_output_domain(Sk&& sk, InFlow&& in_flow, std::string sk_name) {

  auto&& out_port = sk.out_port(in_flow, 0/*last_id offset*/);

  auto&& out_dom = get<0>(out_port).domain();
  auto&& in_dom  = get<0>(in_flow).domain();

  auto&& equal_dom = [&](){ return out_dom.size() == in_dom.size(); };
  std::string title = sk_name + " output port domain test";

  stapl::do_once([&title, &equal_dom](void) {
    stapl::tests::expect(equal_dom()) << title;
  });
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2){
    std::cout << "<exec> <n>" << std::endl;
    exit(1);
  }

  std::size_t n = atol(argv[1]);

  using value_t = int;

  stapl::array<value_t> input(n);
  auto&& input_view = make_array_view(input);

  using namespace skeletons;

  auto&& map_sk = map(stapl::identity<value_t>());

  auto&& flow = flows::make_view_flow<array_view<array<value_t>>>::
                  apply(map_sk, input_view);
  auto&& in_flow_1 = make_tuple(flow);
  auto&& in_flow_2 = make_tuple(flow, flow);

  stapl::plus<value_t>  plus_i;

  test_output_domain(zip(plus_i), in_flow_2, "zip");
  test_output_domain(compose(map_sk, map_sk, map_sk), in_flow_1, "compose");
  test_output_domain(reduce(plus_i), in_flow_1, "reduce");
  test_output_domain(scan(plus_i), in_flow_1, "scan(Jaja)");
  test_output_domain(
    scan<tags::hillis_steele>(plus_i), in_flow_1, "scan(Hillis-Steele)");
  test_output_domain(
    scan<tags::blelloch>(plus_i, 0), in_flow_1, "scan(Blelloch)");

  return EXIT_SUCCESS;
}
