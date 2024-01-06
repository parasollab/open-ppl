/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "../utilities.hpp"
#include "../timer.hpp"

#include <vector>
#include <iostream>


int main(int argc, char *argv[])
{
  size_t size = atoll(argv[1]);

  std::vector<double> acc_samples(10,0.);
  std::vector<double> ad_samples(10,0.);
  std::vector<double> ip_samples(10,0.);
  std::vector<double> ps_samples(10,0.);

  data_t acc_result(0.), ip_result(0.), ad_result(0.);
  bool   acc_correct(true), ip_correct(true), ad_correct(true);

  for (int sanity = 0; sanity != 10; ++sanity)
  {
    std::vector<data_t> x(size);
    std::vector<data_t> y(size);

    //Init
    fill_random(x);
    fill_random(y);


    auto time = start_timer();

    //Apply accumulate
    auto acc_result = std::accumulate(x.begin(), x.end(), 0.);

    acc_samples[sanity] = stop_timer(time);

    acc_correct =
      acc_correct && check_numeric_result(acc_result, x.size(), 0.5);


    time = start_timer();

    auto ip_result =
      std::inner_product(x.begin(), x.end(), y.begin(), 0.);

    ip_samples[sanity] = stop_timer(time);

    ip_correct =
      ip_correct && check_numeric_result(ip_result, x.size(), 0.3333);


    time = start_timer();

    std::partial_sum(x.begin(), x.end(), y.begin());

    ps_samples[sanity] = stop_timer(time);
    time = start_timer();

    std::adjacent_difference(y.begin(), y.end(), x.begin());

    ad_samples[sanity] = stop_timer(time);

    auto ad_result = std::accumulate(x.begin(), x.end(), 0.);

    ad_correct = ad_correct && check_numeric_result(ad_result, x.size(), 0.5);
  }

  report_result("b_accumulate","STL",acc_correct, acc_samples);
  report_result("b_inner_product","STL",ip_correct, ip_samples);

  // partial sum and adjacent different share a correctness check because
  // the operations counter the side effects of one another and restore the
  // container to the initial set of values
  report_result("b_partial_sum","STL",ad_correct, ps_samples);
  report_result("b_adjacent_difference","STL",ad_correct, ad_samples);

  return 0;
}
