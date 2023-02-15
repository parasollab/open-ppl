/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

//////////////////////////////////////////////////////////////////////
/// @file test_domset1D.cc
/// @brief Test that executes a simple @ref map_func over an @ref array_view
///        with @ref domset1D domain.
///
/// There are several test cases differing in the number of disjoint intervals
/// comprising the domset1D domain. There are also two ways of running the
/// tests, one using the default metadata projection
/// (@ref container_metadata_projection), the other using
/// @ref invertible_metadata_projection.
///
/// The default projection performs poorly when the domain consists of multiple
/// intervals. The invertible_metadata_projection has good performance if the
/// number of intervals is less than or equal to the number of locations. In
/// the extreme case of n intervals of size 1, while much slower than the
/// default array_view of same size, it is still significantly faster than
/// the default projection.
///
/// @todo Make the invertible_metadata_projection default for views with
///       domset1D/domainset1D domains.
//////////////////////////////////////////////////////////////////////

#include <iostream>
#include <stapl/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/domains/interval.hpp>
#include <stapl/runtime/counter/default_counters.hpp>
#include "../test_report.hpp"
#include "../confint.hpp"

#define USE_INVERTIBLE_MD_PROJECTION

using array_type = stapl::array<double>;
using domain_type = stapl::domset1D<std::size_t>;
using view_type = stapl::array_view<array_type, domain_type>;

#ifdef USE_INVERTIBLE_MD_PROJECTION
#include <stapl/views/metadata/coarsening_traits.hpp>

namespace stapl {
namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of metadata projection for the view with
///        @ref domset1D domain.
//////////////////////////////////////////////////////////////////////
template<>
struct coarsening_traits<view_type>
{
  template<typename P>
  struct construct_projection
  {
    using type = invertible_metadata_projection<const view_type, P>;
  };
};

} // namespace metadata
} // namespace stapl
#endif

using namespace stapl;

struct wf
{
  template <typename Ref>
  void operator()(Ref&& elem)
  {
    elem += 2;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Execute the test paragraph on given view (incrementing elements
///   in the domain by 2), accumulate the results and compare with the
///   expected sum. Optionally also measure time.
///
/// @param label Label for the test that will be printed with the results.
/// @param view  View to be tested.
/// @param expected_sum Expected result.
/// @param num_cont_it  Number of controller iterations. If the value is
///   greater than one, statistical timing data over given number of runs
///   will be collected and reported.
///
/// @return whether the sum of the elements of the view after executing
///   the paragraph is equal to the expected sum.
//////////////////////////////////////////////////////////////////////
template <typename View>
bool run_test(std::string const& label, View const& view, double expected_sum,
  int num_cont_it)
{
  confidence_interval_controller controller(num_cont_it, num_cont_it);
  counter<default_timer> timer;

  // view over the whole underlying container
  auto av_ref = make_array_view(view.container());

  while(controller.iterate())
  {
    // reset the input view before every run
    fill(av_ref, 0);

    timer.reset();
    timer.start();

    // increment each element in the domain by 2
    stapl::map_func(wf(), view);

    timer.stop();
    controller.push_back(timer.value());
  }
  bool passed = accumulate(av_ref, 0) == expected_sum;

  if (num_cont_it > 1)
    controller.report(label, passed);
  else
    STAPL_TEST_REPORT(passed, label);

  do_once([&] { std::cout << "\n"; });

  return passed;
}

exit_code stapl_main(int argc, char **argv)
{
  if (argc < 2){
    std::cout << "<exec> " << argv[0] << " n [cont_it=1] " << std::endl;
    return EXIT_FAILURE;
  }

  std::size_t n = atol(argv[1]);
  int cont_it = (argc > 2) ? atoi(argv[2]) : 1;

  bool passed = true;

  array_type a(n);
  auto av_ref = make_array_view(a);

  passed &= run_test(
    "Testing domset1D domain (reference results with indexed_domain)",
    av_ref, 2*n, cont_it);

  domain_type contiguous_dom(n);
  view_type av_cont(a, contiguous_dom);

  passed &= run_test("Testing domset1D domain (contiguous domset1D)",
    av_cont, 2*n, cont_it);

  domain_type dom1(0,n/2-1);
  domain_type dom2(n/2+1,n-1);
  domain_type dom = dom1 + dom2;

  view_type av(a, dom);

  passed &= run_test("Testing domset1D domain (2-interval domset1D)",
    av, 2*(n-1), cont_it);

  domain_type dom_even_idx;
  for (size_t i = 0; i < 2*n; i += 2)
    dom_even_idx += i;

  array_type a2(2*n);
  view_type av_even_idx(a2, dom_even_idx);

  passed &= run_test("Testing domset1D domain (n-interval domset1D)",
    av_even_idx, 2*n, cont_it);

  if (not passed)
    return EXIT_FAILURE;

  return EXIT_SUCCESS;
}
