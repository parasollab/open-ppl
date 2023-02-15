/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "../utilities.hpp"

#include <vector>
#include <iostream>

stapl::exit_code stapl_main(int argc, char *argv[])
{
  size_t size = atoll(argv[1]);
  int weak_scaling = 0;
  int data_distribution = 0; // 0=balanced, 1=block, 2=block_cyclic
  stapl::distribution_spec<> data_dist = stapl::balance(size);
  int block_size=2048;

  if (argc > 4)
  {
    weak_scaling = atoi(argv[2]);
    data_distribution = atoi(argv[3]);
    block_size = atoi(argv[4]);
  }

  if (data_distribution == 1)
  {
    data_dist = stapl::block(size,block_size);
  }
  else if (data_distribution == 2)
  {
    data_dist = stapl::block_cyclic(size,block_size);
  }

  std::vector<double> acc_samples(10,0.);
  std::vector<double> ad_samples(10,0.);
  std::vector<double> ip_samples(10,0.);
  std::vector<double> ps_samples(10,0.);

  bool   acc_correct(true), ip_correct(true), ad_correct(true);

  counter_t timer;

  for (int sanity = 0; sanity != 10; ++sanity)
  {
    array_type xcont(data_dist);
    array_type ycont(data_dist);
    array_type_vw x(xcont);
    array_type_vw y(ycont);

    //Init
    fill_random(x, weak_scaling);
    fill_random(y, weak_scaling);

    timer.reset();
    timer.start();

    //Apply accumulate
    auto acc_result = stapl::accumulate(x, 0.);

    acc_samples[sanity] = timer.stop();

    acc_correct =
      acc_correct && check_numeric_result(acc_result, x.size(), 0.5);


    timer.reset();
    timer.start();

    auto ip_result =
      stapl::inner_product(x, y, 0.);

    ip_samples[sanity] = timer.stop();

    ip_correct =
      ip_correct && check_numeric_result(ip_result, x.size(), 0.3333);


    timer.reset();
    timer.start();

    stapl::partial_sum(x, y);

    ps_samples[sanity] = timer.stop();

    timer.reset();
    timer.start();

    stapl::adjacent_difference(y, x);

    ad_samples[sanity] = timer.stop();

    auto ad_result = stapl::accumulate(x, 0.);

    ad_correct = ad_correct && check_numeric_result(ad_result, x.size(), 0.5);
  }

  report_result("b_accumulate","STAPL",acc_correct, acc_samples);
  report_result("b_inner_product","STAPL",ip_correct, ip_samples);

  // partial sum and adjacent different share a correctness check because
  // the operations counter the side effects of one another and restore the
  // container to the initial set of values
  report_result("b_partial_sum","STAPL",ad_correct, ps_samples);
  report_result("b_adjacent_difference","STAPL",ad_correct, ad_samples);

  return EXIT_SUCCESS;
}
