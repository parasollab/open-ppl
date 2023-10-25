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
  size_t size;
  if (argc > 1)
    size = atoll(argv[1]);
  else
    size=13;

  std::vector<double> par_samples(10,0.), sor_samples(10,0.),
    nth_samples(10,0.), min_samples(10,0.), max_samples(10,0.),
    lex_samples(10,0.), sort_samples(10,0.), sta_samples(10,0.);

  bool par_correct(true), sor_correct(true), nth_correct(true),
    min_correct(true), max_correct(true), lex_correct(true),
    sort_correct(true), sta_correct(true);

  // MIN ELEMENT
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);
    fill_random(x);

    auto time = start_timer();

    double result = *(std::min_element(x.begin(), x.end()));

    min_samples[sample] = stop_timer(time);

    // Validate
    auto found = std::find_if(x.begin(), x.end(),
      [result](double i){return i<result;});
    if (found != std::end(x))
      min_correct = false;
  }
  report_result("b_min_element","STL", min_correct, min_samples);

  // MAX ELEMENT
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);
    fill_random(x);

    auto time = start_timer();

    double result = *(std::max_element(x.begin(), x.end()));

    max_samples[sample] = stop_timer(time);

    // Validate
    auto found = std::find_if(x.begin(), x.end(), [result](double i)
      {return i>result;});
    if (found != std::end(x))
      max_correct = false;
  }
  report_result("b_max_element","STL", max_correct, max_samples);

  // LEXICOGRAPHICAL SORT
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);
    std::vector<data_t> y(size);
    fill_random(x);
    y = x;

    auto time = start_timer();

    bool first_less = std::lexicographical_compare(
      x.begin(), x.end(), y.begin(), y.end());

    lex_samples[sample] = stop_timer(time);

    lex_correct = lex_correct && !first_less;
  }
  report_result("b_lex_compare","STL", lex_correct, lex_samples);

  // PARTITION
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);
    fill_random(x);

    auto time = start_timer();

    // Trivial generator function so we don't waste time on that.
    std::partition(x.begin(), x.end(), [](double i){return i < 0.5;});

    par_samples[sample] = stop_timer(time);

    // Validate
    bool greater=false;
    for (size_t i=0; i<size; ++i)
    {
      if (x[i] < 0.5 && greater)
        par_correct = false;
      else if (x[i] >= 0.5)
        greater = true;
    }
  }
  report_result("b_partition","STL", par_correct, par_samples);

  // PARTIAL SORT
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);
    fill_random(x);

    auto time = start_timer();

    std::partial_sort(x.begin(), x.begin()+(size/2), x.end());

    sor_samples[sample] = stop_timer(time);

    // Validate
    sor_correct = sort_correct && std::is_sorted(x.begin(),
      x.begin()+(size/2));
  }
  report_result("b_partial_sort","STL", sor_correct, sor_samples);

  // Nth ELEMENT
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);
    fill_random(x);

    auto time = start_timer();

    std::nth_element(x.begin(), x.begin()+(size/2), x.end());

    nth_samples[sample] = stop_timer(time);

    // Validate
    double nth = x[size/2];
    std::sort(x.begin(), x.end(), std::less<double>());
    nth_correct = nth_correct && (nth == x[size/2]);

  }
  report_result("b_nth_element","STL", nth_correct, nth_samples);

  // SORT
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);
    fill_random(x);

    auto time = start_timer();

    std::sort(x.begin(), x.end(), std::greater<double>());

    sort_samples[sample] = stop_timer(time);

    // Validate
    sort_correct = sort_correct && std::is_sorted(x.begin(), x.end(),
      std::greater<double>());
  }
  report_result("b_sort","STL", sort_correct, sort_samples);

  // STABLE SORT
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);
    fill_random(x);

    auto time = start_timer();

    std::stable_sort(x.begin(), x.end());

    sta_samples[sample] = stop_timer(time);

    // Validate
    sort_correct = sort_correct && std::is_sorted(x.begin(), x.end());
  }
  report_result("b_stable_sort","STL", sta_correct, sta_samples);

  return 0;
}
