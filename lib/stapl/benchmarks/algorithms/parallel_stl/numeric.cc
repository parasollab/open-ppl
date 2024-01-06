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
  int num_threads = atoi( argv[2] );

  if (num_threads > 1)
  {
    // Force the use of parallel STL versions of the algorithms 
    __gnu_parallel::_Settings s;
    s.algorithm_strategy = __gnu_parallel::force_parallel;
    __gnu_parallel::_Settings::set(s);
  }

  omp_set_num_threads(num_threads);

  std::vector<double> acc_samples(10,0.);
  std::vector<double> ad_samples(10,0.);
  std::vector<double> ip_samples(10,0.);
  std::vector<double> ps_samples(10,0.);

  data_t acc_result(0.), ip_result(0.), ad_result(0.);
  bool   acc_correct(true), ip_correct(true), ad_correct(true);

  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);
    std::vector<data_t> y(size);

    //Init
    fill_random(x);
    fill_random(y);


    auto time = start_timer();

    //Apply accumulate
    auto acc_result = __gnu_parallel::accumulate(x.begin(), x.end(), 0.);

    acc_samples[sample] = stop_timer(time);

    acc_correct =
      acc_correct && check_numeric_result(acc_result, x.size(), 0.5);


    time = start_timer();

    auto ip_result =
      __gnu_parallel::inner_product(x.begin(), x.end(), y.begin(), 0.);

    ip_samples[sample] = stop_timer(time);

    ip_correct =
      ip_correct && check_numeric_result(ip_result, x.size(), 0.3333);


    time = start_timer();

    __gnu_parallel::partial_sum(x.begin(), x.end(), y.begin());

    ps_samples[sample] = stop_timer(time);
    time = start_timer();

    __gnu_parallel::adjacent_difference(y.begin(), y.end(), x.begin());

    ad_samples[sample] = stop_timer(time);

    auto ad_result = __gnu_parallel::accumulate(x.begin(), x.end(), 0.);

    ad_correct = ad_correct && check_numeric_result(ad_result, x.size(), 0.5);

  }

  report_result("b_accumulate","gnu_parallel",acc_correct, acc_samples);
  report_result("b_inner_product","gnu_parallel",ip_correct, ip_samples);

  // partial sum and adjacent different share a correctness check because
  // the operations counter the side effects of one another and restore the
  // container to the initial set of values
  report_result("b_partial_sum","gnu_parallel",ad_correct, ps_samples);
  report_result("b_adjacent_difference","gnu_parallel",ad_correct, ad_samples);

  return 0;
}
