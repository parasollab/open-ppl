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

void test_adjacent_find(std::vector<data_t>& x, std::vector<double>& samples0,
       std::vector<double>& samples25, std::vector<double>& samples75,
       bool& correct, int sample);

void test_count(std::vector<data_t>& x, std::vector<double>& samples,
       bool& correct, int sample);

void test_count_if(std::vector<data_t>& x, std::vector<double>& samples,
       bool& correct, int sample);

void test_find(std::vector<data_t>& x, std::vector<double>& samples0,
       std::vector<double>& samples25, std::vector<double>& samples75,
       bool& correct, int sample);

void test_find_if(std::vector<data_t>& x, std::vector<double>& samples0,
       std::vector<double>& samples25, std::vector<double>& samples75,
       bool& correct, int sample);

void test_find_first_of(std::vector<data_t>& x, std::vector<double>& samples0,
       std::vector<double>& samples25, std::vector<double>& samples75,
       bool& correct, int sample);

void test_search_search_n(std::vector<data_t>& x,
       std::vector<double>& search_samples0,
       std::vector<double>& search_samples25,
       std::vector<double>& search_samples75,
       std::vector<double>& search_n_samples0,
       std::vector<double>& search_n_samples25,
       std::vector<double>& search_n_samples75,
       bool& search_correct, bool search_n_correct, int sample);

void test_mismatch_equal(std::vector<data_t>& x, std::vector<data_t>& y,
       std::vector<double>& mismatch_samples0,
       std::vector<double>& mismatch_samples25,
       std::vector<double>& mismatch_samples75,
       std::vector<double>& equal_samples0,
       std::vector<double>& equal_samples25,
       std::vector<double>& equal_samples75, bool& mismatch_correct,
       bool& equal_correct, int sample);


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

  std::vector<double> for_each_samples(10,0.),
    find_samples0(10,0.), find_samples25(10,0.), find_samples75(10,0.),
    find_if_samples0(10,0.), find_if_samples25(10,0.), find_if_samples75(10,0.),
    find_first_of_samples0(10,0.), find_first_of_samples25(10,0.),
    find_first_of_samples75(10,0.), adjacent_find_samples0(10,0.),
    adjacent_find_samples25(10,0.), adjacent_find_samples75(10,0.),
    count_samples(10,0.), count_if_samples(10,0.),
    mismatch_samples0(10,0.), mismatch_samples25(10,0.),
    mismatch_samples75(10,0.), equal_samples0(10,0.), equal_samples25(10,0.),
    equal_samples75(10,0.), search_samples0(10,0.), search_samples25(10,0.),
    search_samples75(10,0.), search_n_samples0(10,0.),
    search_n_samples25(10,0.), search_n_samples75(10,0.);

  bool for_each_correct(true), find_correct(true), find_if_correct(true),
       find_first_of_correct(true), adjacent_find_correct(true),
       count_correct(true), count_if_correct(true), mismatch_correct(true),
       equal_correct(true), search_correct(true), search_n_correct(true);

  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);
    std::vector<data_t> y(size);

    //Init
    fill_random(x);
    fill_random(y);

    auto time = start_timer();

    __gnu_parallel::for_each(x.begin(), x.end(), [](data_t &n){ n = n*n; });

    for_each_samples[sample] = stop_timer(time);

    auto for_each_result = __gnu_parallel::accumulate(x.begin(), x.end(), 0.);

    for_each_correct = for_each_correct &&
      check_numeric_result(for_each_result, x.size(), 0.3333);

    test_find(x, find_samples0, find_samples25, find_samples75,
              find_correct, sample);

    test_find_if(x, find_if_samples0, find_if_samples25, find_if_samples75,
                 find_if_correct, sample);

    test_find_first_of(x, find_first_of_samples0, find_first_of_samples25,
                       find_first_of_samples75, find_first_of_correct, sample);

    test_adjacent_find(x, adjacent_find_samples0, adjacent_find_samples25,
                       adjacent_find_samples75, adjacent_find_correct, sample);

    test_search_search_n(x, search_samples0, search_samples25, search_samples75,
                         search_n_samples0, search_n_samples25,
                         search_n_samples75, search_correct, search_n_correct,
                         sample);

    test_count(x, count_samples, count_correct, sample);

    test_count_if(x, count_if_samples, count_if_correct, sample);

    test_mismatch_equal(x, y, mismatch_samples0, mismatch_samples25,
                        mismatch_samples75, equal_samples0, equal_samples25,
                        equal_samples75, mismatch_correct, equal_correct,
                        sample);
  }

  report_result("b_for_each","gnu_parallel",for_each_correct, for_each_samples);
  report_result("b_find-No -- not present","gnu_parallel",
                find_correct, find_samples0);
  report_result("b_find-25 -- elem at 25%","gnu_parallel",
                find_correct, find_samples25);
  report_result("b_find-75 -- elem at 75%","gnu_parallel",
                find_correct, find_samples75);
  report_result("b_find_if-No -- not present","gnu_parallel",
                find_if_correct, find_if_samples0);
  report_result("b_find_if-25 -- elem at 25%","gnu_parallel",
                find_if_correct, find_if_samples25);
  report_result("b_find_if-75 -- elem at 75%","gnu_parallel",
                find_if_correct, find_if_samples75);
  report_result("b_find_first_of-No -- not present","gnu_parallel",
                find_first_of_correct, find_first_of_samples0);
  report_result("b_find_first_of-25 -- elem at 25%","gnu_parallel",
                find_first_of_correct, find_first_of_samples25);
  report_result("b_find_first_of-75 -- elem at 75%","gnu_parallel",
                find_first_of_correct, find_first_of_samples75);
  report_result("b_adjacent_find-No -- not present","gnu_parallel",
                adjacent_find_correct, adjacent_find_samples0);
  report_result("b_adjacent_find-25 -- elem at 25%","gnu_parallel",
                adjacent_find_correct, adjacent_find_samples25);
  report_result("b_adjacent_find-75 -- elem at 75%","gnu_parallel",
                adjacent_find_correct, adjacent_find_samples75);
  report_result("b_count","gnu_parallel",count_correct, count_samples);
  report_result("b_count_if","gnu_parallel",count_if_correct, count_if_samples);
  report_result("b_equal-all -- all equal","gnu_parallel",
                equal_correct, equal_samples0);
  report_result("b_equal-25 -- unequal at 25%","gnu_parallel",
                equal_correct, equal_samples25);
  report_result("b_equal-75 -- unequal at 75%","gnu_parallel",
                equal_correct, equal_samples75);
  report_result("b_mismatch-No -- no mismatch","gnu_parallel",
                mismatch_correct, mismatch_samples0);
  report_result("b_mismatch-25 -- mismatch at 25%","gnu_parallel",
                mismatch_correct, mismatch_samples25);
  report_result("b_mismatch-75 -- mismatch at 75%","gnu_parallel",
                mismatch_correct, mismatch_samples75);
  report_result("b_search-No -- no substring","gnu_parallel",
                search_correct, search_samples0);
  report_result("b_search-25 -- substring at 25%","gnu_parallel",
                search_correct, search_samples25);
  report_result("b_search-75 -- substring at 75%","gnu_parallel",
                search_correct, search_samples75);
  report_result("b_search_n-No -- no substring","gnu_parallel",
                search_correct, search_n_samples0);
  report_result("b_search_n-25 -- substring at 25%","gnu_parallel",
                search_correct, search_n_samples25);
  report_result("b_search_n-75 -- substring at 75%","gnu_parallel",
                search_correct, search_n_samples75);

  return 0;
}


void test_find(std::vector<data_t>& x, std::vector<double>& samples0,
       std::vector<double>& samples25,
       std::vector<double>& samples75,
       bool& correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  // Case where element is not found
  auto time = start_timer();

  auto result = __gnu_parallel::find(x.begin(), x.end(), data_val);

  samples0[sample] = stop_timer(time);

  correct = correct && result == x.end();

  // Case where element is 25% in to container
  set_known_value(x, quarter_idx);

  time = start_timer();

  result = __gnu_parallel::find(x.begin(), x.end(), data_val);

  samples25[sample] = stop_timer(time);

  correct = correct && std::distance(x.begin(), result) == quarter_idx;

  clear_known_value(x, quarter_idx);

  // Case where element is 75% in to container
  set_known_value(x, three_quarter_idx);

  time = start_timer();

  result = __gnu_parallel::find(x.begin(), x.end(), data_val);

  samples75[sample] = stop_timer(time);

  correct = correct && std::distance(x.begin(), result) == three_quarter_idx;

  clear_known_value(x, three_quarter_idx);
}


void test_find_if(std::vector<data_t>& x, std::vector<double>& samples0,
       std::vector<double>& samples25,
       std::vector<double>& samples75,
       bool& correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  // Case where element is not found
  auto time = start_timer();

  auto result =
    __gnu_parallel::find_if(x.begin(), x.end(), equal_known(data_val));

  samples0[sample] = stop_timer(time);

  correct = correct && result == x.end();

  // Case where element is 25% in to container
  set_known_value(x, quarter_idx);

  time = start_timer();

  result = __gnu_parallel::find_if(x.begin(), x.end(), equal_known(data_val));

  samples25[sample] = stop_timer(time);

  correct = correct && std::distance(x.begin(), result) == quarter_idx;

  clear_known_value(x, quarter_idx);

  // Case where element is 75% in to container
  set_known_value(x, three_quarter_idx);

  time = start_timer();

  result = __gnu_parallel::find_if(x.begin(), x.end(), equal_known(data_val));

  samples75[sample] = stop_timer(time);

  correct = correct && std::distance(x.begin(), result) == three_quarter_idx;

  clear_known_value(x, three_quarter_idx);
}

void test_find_first_of(std::vector<data_t>& x, std::vector<double>& samples0,
       std::vector<double>& samples25, std::vector<double>& samples75,
       bool& correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  // Declare second vector of values to be used in find_first_of call, default
  // constructing the elements and then setting the last element equal to the
  // known value.
  std::vector<data_t> search_space(25);
  search_space[24] = data_val;

  // Case where element is not found
  auto time = start_timer();

  auto result = __gnu_parallel::find_first_of(x.begin(), x.end(),
                  search_space.begin(), search_space.end());

  samples0[sample] = stop_timer(time);

  correct = correct && result == x.end();

  // Case where element is 25% in to container
  set_known_value(x, quarter_idx);

  time = start_timer();

  result = __gnu_parallel::find_first_of(x.begin(), x.end(),
             search_space.begin(), search_space.end());

  samples25[sample] = stop_timer(time);

  correct = correct && std::distance(x.begin(), result) == quarter_idx;

  clear_known_value(x, quarter_idx);

  // Case where element is 75% in to container
  set_known_value(x, three_quarter_idx);

  time = start_timer();

  result = __gnu_parallel::find_first_of(x.begin(), x.end(),
             search_space.begin(), search_space.end());

  samples75[sample] = stop_timer(time);

  correct = correct && std::distance(x.begin(), result) == three_quarter_idx;

  clear_known_value(x, three_quarter_idx);
}


void test_adjacent_find(std::vector<data_t>& x, std::vector<double>& samples0,
       std::vector<double>& samples25, std::vector<double>& samples75,
       bool& correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  // Case where element is not found
  auto time = start_timer();

  auto result = __gnu_parallel::adjacent_find(x.begin(), x.end());

  samples0[sample] = stop_timer(time);

  correct = correct && result == x.end();

  // Case where element is 25% in to container
  set_known_value(x, quarter_idx);
  set_known_value(x, quarter_idx+1);

  time = start_timer();

  result = __gnu_parallel::adjacent_find(x.begin(), x.end());

  samples25[sample] = stop_timer(time);

  correct = correct && std::distance(x.begin(), result) == quarter_idx;

  clear_known_value(x, quarter_idx, 2);

  // Case where element is 75% in to container
  set_known_value(x, three_quarter_idx);
  set_known_value(x, three_quarter_idx+1);

  time = start_timer();

  result = __gnu_parallel::adjacent_find(x.begin(), x.end());

  samples75[sample] = stop_timer(time);

  correct = correct && std::distance(x.begin(), result) == three_quarter_idx;

  clear_known_value(x, three_quarter_idx, 2);
}


void test_count(std::vector<data_t>& x, std::vector<double>& samples,
       bool& correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  set_known_value(x, quarter_idx);
  set_known_value(x, three_quarter_idx);

  auto time = start_timer();

  auto result = __gnu_parallel::count(x.begin(), x.end(), data_val);

  samples[sample] = stop_timer(time);

  clear_known_value(x, quarter_idx);
  clear_known_value(x, three_quarter_idx);

  correct = correct && result == 2;
}


void test_count_if(std::vector<data_t>& x, std::vector<double>& samples,
       bool& correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  set_known_value(x, quarter_idx);
  set_known_value(x, three_quarter_idx);

  auto time = start_timer();

  auto result =
    __gnu_parallel::count_if(x.begin(), x.end(), equal_known(data_val));

  samples[sample] = stop_timer(time);

  clear_known_value(x, quarter_idx);
  clear_known_value(x, three_quarter_idx);

  correct = correct && result == 2;
}


void test_mismatch_equal(std::vector<data_t>& x, std::vector<data_t>& y,
       std::vector<double>& mismatch_samples0,
       std::vector<double>& mismatch_samples25,
       std::vector<double>& mismatch_samples75,
       std::vector<double>& equal_samples0,
       std::vector<double>& equal_samples25,
       std::vector<double>& equal_samples75,
       bool& mismatch_correct, bool& equal_correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  // Ensure the sequences are equal
  std::copy(x.begin(), x.end(), y.begin());

  // Evaluate algorithms when sequences are equal
  auto time = start_timer();

  bool equal_result =
    __gnu_parallel::equal(x.begin(), x.end(), y.begin());

  equal_samples0[sample] = stop_timer(time);

  equal_correct = equal_correct && equal_result;

  time = start_timer();

  auto mismatch_result =
    __gnu_parallel::mismatch(x.begin(), x.end(), y.begin());

  mismatch_samples0[sample] = stop_timer(time);

  mismatch_correct = mismatch_correct && mismatch_result.first == x.end() &&
                     mismatch_result.second == y.end();

  // Set mismatch 75% in to y
  set_known_value(y, three_quarter_idx);

  time = start_timer();

  equal_result =
    __gnu_parallel::equal(x.begin(), x.end(), y.begin());

  equal_samples75[sample] = stop_timer(time);

  equal_correct = equal_correct && !equal_result;

  time = start_timer();

  mismatch_result =
    __gnu_parallel::mismatch(x.begin(), x.end(), y.begin());

  mismatch_samples75[sample] = stop_timer(time);

  mismatch_correct = mismatch_correct &&
    std::distance(x.begin(), mismatch_result.first) == three_quarter_idx &&
    std::distance(y.begin(), mismatch_result.second) == three_quarter_idx;

  // Set mismatch 25% in to x
  set_known_value(x, quarter_idx);

  time = start_timer();

  equal_result =
    __gnu_parallel::equal(x.begin(), x.end(), y.begin());

  equal_samples25[sample] = stop_timer(time);

  equal_correct = equal_correct && !equal_result;

  time = start_timer();

  mismatch_result =
    __gnu_parallel::mismatch(x.begin(), x.end(), y.begin());

  mismatch_samples25[sample] = stop_timer(time);

  mismatch_correct = mismatch_correct &&
    std::distance(x.begin(), mismatch_result.first) == quarter_idx &&
    std::distance(y.begin(), mismatch_result.second) == quarter_idx;
}

void test_search_search_n(std::vector<data_t>& x,
       std::vector<double>& search_samples0,
       std::vector<double>& search_samples25,
       std::vector<double>& search_samples75,
       std::vector<double>& search_n_samples0,
       std::vector<double>& search_n_samples25,
       std::vector<double>& search_n_samples75,
       bool& search_correct, bool search_n_correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  // Declare second vector of values to be used in the search call.
  std::vector<data_t> substring(25, data_val);

  // Case where element is not found
  auto time = start_timer();

  auto result = __gnu_parallel::search(x.begin(), x.end(),
                  substring.begin(), substring.end());

  search_samples0[sample] = stop_timer(time);

  search_correct = search_correct && result == x.end();

  time = start_timer();

  result = __gnu_parallel::search_n(x.begin(), x.end(), 25, data_val);

  search_n_samples0[sample] = stop_timer(time);

  search_n_correct = search_n_correct && result == x.end();

  // Case where element is 25% in to container
  for (int i = 0; i != 25; ++i)
    set_known_value(x, quarter_idx+i);

  time = start_timer();

  result = __gnu_parallel::search(x.begin(), x.end(),
             substring.begin(), substring.end());

  search_samples25[sample] = stop_timer(time);

  search_correct = search_correct &&
    std::distance(x.begin(), result) == quarter_idx;

  time = start_timer();

  result = __gnu_parallel::search_n(x.begin(), x.end(), 25, data_val);

  search_n_samples25[sample] = stop_timer(time);

  search_n_correct = search_n_correct &&
    std::distance(x.begin(), result) == quarter_idx;

  clear_known_value(x, quarter_idx, 25);

  // Case where element is 75% in to container
  for (int i = 0; i != 25; ++i)
    set_known_value(x, three_quarter_idx+i);

  time = start_timer();

  result = __gnu_parallel::search(x.begin(), x.end(),
             substring.begin(), substring.end());

  search_samples75[sample] = stop_timer(time);

  search_correct = search_correct &&
    std::distance(x.begin(), result) == three_quarter_idx;

  time = start_timer();

  result = __gnu_parallel::search_n(x.begin(), x.end(), 25, data_val);

  search_n_samples75[sample] = stop_timer(time);

  search_n_correct = search_n_correct &&
    std::distance(x.begin(), result) == three_quarter_idx;

  clear_known_value(x, three_quarter_idx, 25);
}
