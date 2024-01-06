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
#include <stapl/array.hpp>

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

  std::vector<double> par_samples(10,0.), sor_samples(10,0.),
    nth_samples(10,0.), min_samples(10,0.), max_samples(10,0.),
    lex_samples(10,0.), sort_samples(10,0.), sta_samples(10,0.);

  bool par_correct(true), sor_correct(true), nth_correct(true),
    min_correct(true), max_correct(true), lex_correct(true),
    sort_correct(true), sta_correct(true);

  counter_t timer;

  // MIN ELEMENT
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont(data_dist);
    array_type_vw x(xcont);

    fill_random(x, weak_scaling);
    timer.reset();
    timer.start();

    auto result = stapl::min_element(x);

    min_samples[sample] = timer.stop();

    // Validate
    auto result0 = stapl::is_null_reference(
      stapl::find_if(x, std::bind(std::less<double>(),
      std::placeholders::_1, result)));

    min_correct = min_correct && result0;
  }
  report_result("b_min_element","STAPL", min_correct, min_samples);

  // MAX ELEMENT
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont(data_dist);
    array_type_vw x(xcont);

    fill_random(x, weak_scaling);
    timer.reset();
    timer.start();

    auto result = stapl::max_element(x);

    max_samples[sample] = timer.stop();

    // Validate
    auto result0 = stapl::is_null_reference(
      stapl::find_if(x, std::bind(std::greater<double>(),
      std::placeholders::_1, result)));

    max_correct = max_correct && result0;
  }

  report_result("b_max_element","STAPL", max_correct, max_samples);

  // LEXICOGRAPHICAL COMPARE
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont(data_dist);
    array_type_vw x(xcont);

    fill_random(x, weak_scaling);
    array_type ycont(xcont);
    array_type_vw y(ycont);
    timer.reset();
    timer.start();

    bool first_less = stapl::lexicographical_compare(x, y);

    lex_samples[sample] = timer.stop();

    lex_correct = lex_correct && !first_less;
  }
  report_result("b_lex_compare","STAPL", lex_correct, lex_samples);

  // PARTITION
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont(data_dist);
    array_type_vw x(xcont);

    fill_random(x, weak_scaling);
    timer.reset();
    timer.start();

    stapl::partition(x, [](double i){return i < 0.5;});

    par_samples[sample] = timer.stop();

    // Validate
    stapl::do_once([&xcont, &par_correct, &size]{
      bool greater=false;
      for (size_t i=0; i<size; ++i)
      {
        if (xcont[i] < 0.5 && greater)
          par_correct = false;
        else if (xcont[i] >= 0.5)
          greater = true;
      }
    });
  }
  report_result("b_partition","STAPL", par_correct, par_samples);

  // PARTIAL SORT
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont(data_dist);
    array_type_vw x(xcont);

    fill_random(x, weak_scaling);
    timer.reset();
    timer.start();

    stapl::partial_sort(x, x.begin()+(size/2));

    sor_samples[sample] = timer.stop();

    // Validate
    stapl::do_once([&xcont, &sor_correct, &size]{
      double prev = xcont[0];
      for (size_t i=1; i<size/2; ++i)
      {
        if (xcont[i] < prev)
          sor_correct = false;
        prev = xcont[i];
      }
    });
  }
  report_result("b_partial_sort","STAPL", sor_correct, sor_samples);

  // Nth ELEMENT
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont(data_dist);
    array_type_vw x(xcont);

    fill_random(x, weak_scaling);
    timer.reset();
    timer.start();

    stapl::nth_element(x, x.begin()+(size/2));

    nth_samples[sample] = timer.stop();

    // Validate
    double nth = x[size/2];
    stapl::sort(x, std::less<double>());
    stapl::do_once([&nth_correct, &size, &x, &nth] {
      nth_correct = nth_correct && (nth == x[size/2]);
    });
  }
  report_result("b_nth_element","STAPL", nth_correct, nth_samples);

  // SORT
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont(data_dist);
    array_type_vw x(xcont);

    fill_random(x, weak_scaling);
    timer.reset();
    timer.start();

    stapl::sort(x, std::greater<double>());

    sort_samples[sample] = timer.stop();

    // Validate
    sort_correct = sort_correct && stapl::is_sorted(x, std::greater<double>());
  }
  report_result("b_sort","STAPL", sort_correct, sort_samples);

  // STABLE SORT
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont(data_dist);
    array_type_vw x(xcont);

    fill_random(x, weak_scaling);
    timer.reset();
    timer.start();

    stapl::stable_sort(x);

    sta_samples[sample] = timer.stop();

    // Validate
    sta_correct = sta_correct && stapl::is_sorted(x);
  }
  report_result("b_stable_sort","STAPL", sta_correct, sta_samples);

  return EXIT_SUCCESS;
}
