/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <iostream>
#include <vector>
#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/utility/use_default.hpp>
#include <stapl/utility/do_once.hpp>

//includes from skeletons
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/transformations/coarse/butterfly.hpp>
#include <stapl/skeletons/transformations/coarse/scan_reduce.hpp>
#include <stapl/skeletons/functional/butterfly.hpp>
#include <stapl/skeletons/functional/sink.hpp>
#include <stapl/skeletons/scan_reduce.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include "../test_report.hpp"

#include <boost/bind.hpp>

using namespace std;
using namespace stapl;

namespace  {

bool is_power_of_two(unsigned int x)
{
  while (((x % 2) == 0) && x > 1)
    x /= 2;
  return (x == 1);
}

template <bool isExclusive = false, typename View>
void verify_scan(View&& input_view, View&& output_view, std::string title)
{
#ifdef VALIDATE_RESULTS
  do_once([&input_view, &output_view, &title](void) {
    using value_t = typename std::decay<View>::type::value_type;
    std::vector<value_t> result_test(input_view.size());
    std::partial_sum(input_view.begin(), input_view.end(), result_test.begin());
    bool is_valid = std::equal(output_view.begin() + (isExclusive ? 1 : 0),
                               output_view.end(),
                               result_test.begin());

    is_valid &= isExclusive ? (*(output_view.begin()) == (std::size_t)0) : true;
    STAPL_TEST_REPORT(is_valid, title);
  });
#endif
}

} // namespace


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: exe n" << std::endl;
    exit(1);
  }

  size_t n = atol(argv[1]);

  using namespace skeletons;
  // typedef std::size_t               value_t;
  typedef int              value_t;
  typedef stapl::array<value_t>     container_t;
  typedef array_view<container_t>   view_t;
  typedef stapl::plus<value_t>      op_t;

  container_t  input_array(n);
  view_t       input_view(input_array);
  container_t  output_array(n, 0);
  view_t       output_view(output_array);

  //initialize the input view
  stapl::copy(counting_view<value_t>(n, value_t(4)), input_view);

  value_t res = stapl::reduce(input_view, op_t());
 #ifdef VALIDATE_RESULTS
  do_once([&input_view, res](void) {
     value_t red_value = std::accumulate(input_view.begin(),
                                         input_view.end(), 0);
     STAPL_TEST_REPORT(red_value == res, "coarse_reduce_to_loc(+)");
   });
 #endif

  stapl::scan<tags::scan<tags::blelloch, tags::exclusive>, tags::naive>(
    input_view, output_view, op_t());
  verify_scan<true>(input_view, output_view,
                    "coarse(scan(blelloch)(+), naive)");
  stapl::scan<tags::scan<tags::blelloch, tags::exclusive>, stapl::use_default>(
    input_view, output_view, op_t());
  verify_scan<true>(input_view, output_view,
                    "coarse(scan(blelloch)(+))");

  stapl::scan<tags::scan<tags::hillis_steele>, stapl::use_default>(
    input_view, output_view, op_t());
  verify_scan(input_view, output_view,
              "coarse(scan(hillis-steele)(+))");

  stapl::scan<tags::scan<tags::binomial>, stapl::use_default>(
    input_view, output_view, op_t());
  verify_scan(input_view, output_view,
              "coarse(scan(binomial)(+))");

  stapl::scan<tags::scan<tags::jaja>, stapl::use_default>(
    input_view, output_view, op_t());
  verify_scan(input_view, output_view,
              "coarse(scan(jaja)(+))");


  value_t result = scan_reduce(input_view, output_view, stapl::plus<value_t>());

  verify_scan(input_view, output_view,
              "coarse(scan_reduce(binomial)(+)) scan result");

 #ifdef VALIDATE_RESULTS
  do_once([&input_view, result](void) {
     value_t red_value = std::accumulate(input_view.begin(),
                                         input_view.end(), 0);
     STAPL_TEST_REPORT(
      red_value == result,
      "coarse(scan_reduce(binomial)(+)) reduce result");
   });
 #endif



  result = scan_reduce(input_view, output_view, stapl::plus<value_t>(), true);

  verify_scan<true>(input_view, output_view,
              "coarse(scan_reduce(blelloch)(+)) scan result");

 #ifdef VALIDATE_RESULTS
  do_once([&input_view, result](void) {
     value_t red_value = std::accumulate(input_view.begin(),
                                         input_view.end(), 0);
     STAPL_TEST_REPORT(
      red_value == result,
      "coarse(scan_reduce(blelloch)(+)) reduce result");
   });
 #endif
  if (is_power_of_two(get_num_locations()) && is_power_of_two(n)) {
    skeletons::execute(
      skeletons::execution_params(default_coarsener()),
      sink<value_t>(
        coarse(skeletons::butterfly(op_t())),
        coarse(skeletons::zip(stapl::assign<value_t>()))),
      input_view, output_view);

#ifdef VALIDATE_RESULTS
    do_once([&input_view, &output_view](void) {
      value_t red_value = std::accumulate(input_view.begin(),
                                          input_view.end(), 0);
      bool is_valid = std::accumulate(
                        boost::make_transform_iterator(
                          output_view.begin(),
                          boost::bind(std::equal_to<value_t>(), _1, red_value)),
                        boost::make_transform_iterator(
                          output_view.begin(),
                          boost::bind(std::equal_to<value_t>(), _1, red_value)),
                          true,
                          std::logical_and<bool>());
      STAPL_TEST_REPORT(is_valid, "coarse(butterfly(+))");
    });
#endif

  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    sink<value_t>(
      coarse(skeletons::reverse_butterfly(op_t())),
      coarse(skeletons::zip(stapl::assign<value_t>()))),
    input_view, output_view);

#ifdef VALIDATE_RESULTS
    do_once([&input_view, &output_view](void) {
      value_t red_value = std::accumulate(input_view.begin(),
                                          input_view.end(), 0);
      bool is_valid = std::accumulate(
                        boost::make_transform_iterator(
                          output_view.begin(),
                          boost::bind(std::equal_to<value_t>(), _1, red_value)),
                        boost::make_transform_iterator(
                          output_view.begin(),
                          boost::bind(std::equal_to<value_t>(), _1, red_value)),
                          true,
                          std::logical_and<bool>());
      STAPL_TEST_REPORT(is_valid, "coarse(reverse_butterfly(+))");
    });
#endif
  }
  return EXIT_SUCCESS;
}
