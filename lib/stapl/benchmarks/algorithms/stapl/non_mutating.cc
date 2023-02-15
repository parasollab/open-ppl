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

using container_t = array_type;
using view_t      = stapl::array_view<container_t>;


void test_adjacent_find(view_t& x, std::vector<double>& samples0,
       std::vector<double>& samples25, std::vector<double>& samples75,
       bool& correct, int sample);

void test_find(view_t& x, std::vector<double>& samples0,
       std::vector<double>& samples25, std::vector<double>& samples75,
       bool& correct, int sample);

void test_find_if(view_t& x, std::vector<double>& samples0,
       std::vector<double>& samples25, std::vector<double>& samples75,
       bool& correct, int sample);

void test_find_first_of(view_t& x, std::vector<double>& samples0,
       std::vector<double>& samples25, std::vector<double>& samples75,
       bool& correct, int sample);

void test_search_search_n(view_t& x,
       std::vector<double>& search_samples0,
       std::vector<double>& search_samples25,
       std::vector<double>& search_samples75,
       std::vector<double>& search_n_samples0,
       std::vector<double>& search_n_samples25,
       std::vector<double>& search_n_samples75,
       bool& search_correct, bool search_n_correct, int sample);

void test_count(view_t& x, std::vector<double>& samples,
       bool& correct, int sample);

void test_count_if(view_t& x, std::vector<double>& samples,
       bool& correct, int sample);

void test_mismatch_equal(view_t& x, view_t& y,
       std::vector<double>& mismatch_samples0,
       std::vector<double>& mismatch_samples25,
       std::vector<double>& mismatch_samples75,
       std::vector<double>& equal_samples0,
       std::vector<double>& equal_samples25,
       std::vector<double>& equal_samples75, bool& mismatch_correct,
       bool& equal_correct, int sample);


struct square
{
  template<typename Ref>
  void operator()(Ref&& val) const
  { val = val*val; }
};


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

  counter_t timer;

  for (int sample = 0; sample != 10; ++sample)
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

    stapl::for_each(x, square());

    for_each_samples[sample] = timer.stop();

    auto for_each_result = stapl::accumulate(x, 0.);

    for_each_correct = for_each_correct &&
      check_numeric_result(for_each_result, x.size(), 0.3333);

    test_find(x, find_samples0, find_samples25, find_samples75,
              find_correct, sample);

    test_find_if(x, find_if_samples0, find_if_samples25, find_if_samples75,
                 find_if_correct, sample);

    test_find_first_of(x, find_first_of_samples0, find_first_of_samples25,
                       find_first_of_samples75, find_first_of_correct, sample);

    test_search_search_n(x, search_samples0, search_samples25, search_samples75,
                         search_n_samples0, search_n_samples25,
                         search_n_samples75, search_correct, search_n_correct,
                         sample);

    test_adjacent_find(x, adjacent_find_samples0, adjacent_find_samples25,
                       adjacent_find_samples75, adjacent_find_correct, sample);

    test_count(x, count_samples, count_correct, sample);

    test_count_if(x, count_if_samples, count_if_correct, sample);

    test_mismatch_equal(x, y, mismatch_samples0, mismatch_samples25,
                        mismatch_samples75, equal_samples0, equal_samples25,
                        equal_samples75, mismatch_correct, equal_correct,
                        sample);
  }

  report_result("b_for_each","STAPL",for_each_correct, for_each_samples);
  report_result("b_find-No -- not present","STAPL",
                find_correct, find_samples0);
  report_result("b_find-25 -- elem at 25%","STAPL",
                find_correct, find_samples25);
  report_result("b_find-75 -- elem at 75%","STAPL",
                find_correct, find_samples75);
  report_result("b_find_if-No -- not present","STAPL",
                find_if_correct, find_if_samples0);
  report_result("b_find_if-25 -- elem at 25%","STAPL",
                find_if_correct, find_if_samples25);
  report_result("b_find_if-75 -- elem at 75%","STAPL",
                find_if_correct, find_if_samples75);
  report_result("b_find_first_of-No -- not present","STAPL",
                find_first_of_correct, find_first_of_samples0);
  report_result("b_find_first_of-25 -- elem at 25%","STAPL",
                find_first_of_correct, find_first_of_samples25);
  report_result("b_find_first_of-75 -- elem at 75%","STAPL",
                find_first_of_correct, find_first_of_samples75);
  report_result("b_adjacent_find-No -- not present","STAPL",
                adjacent_find_correct, adjacent_find_samples0);
  report_result("b_adjacent_find-25 -- elem at 25%","STAPL",
                adjacent_find_correct, adjacent_find_samples25);
  report_result("b_adjacent_find-75 -- elem at 75%","STAPL",
                adjacent_find_correct, adjacent_find_samples75);
  report_result("b_count","STAPL",count_correct, count_samples);
  report_result("b_count_if","STAPL",count_if_correct, count_if_samples);
  report_result("b_equal-all -- all equal","STAPL",
                equal_correct, equal_samples0);
  report_result("b_equal-25 -- unequal at 25%","STAPL",
                equal_correct, equal_samples25);
  report_result("b_equal-75 -- unequal at 75%","STAPL",
                equal_correct, equal_samples75);
  report_result("b_mismatch-No -- no mismatch","STAPL",
                mismatch_correct, mismatch_samples0);
  report_result("b_mismatch-25 -- mismatch at 25%","STAPL",
                mismatch_correct, mismatch_samples25);
  report_result("b_mismatch-75 -- mismatch at 75%","STAPL",
                mismatch_correct, mismatch_samples75);
  report_result("b_search-No -- no substring","STAPL",
                search_correct, search_samples0);
  report_result("b_search-25 -- substring at 25%","STAPL",
                search_correct, search_samples25);
  report_result("b_search-75 -- substring at 75%","STAPL",
                search_correct, search_samples75);
  report_result("b_search_n-No -- no substring","STAPL",
                search_correct, search_n_samples0);
  report_result("b_search_n-25 -- substring at 25%","STAPL",
                search_correct, search_n_samples25);
  report_result("b_search_n-75 -- substring at 75%","STAPL",
                search_correct, search_n_samples75);

  return EXIT_SUCCESS;
}


void test_find(view_t& x, std::vector<double>& samples0,
       std::vector<double>& samples25,
       std::vector<double>& samples75,
       bool& correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  counter_t timer;

  // Case where element is not found
  timer.reset();
  timer.start();

  auto result0 = stapl::find(x, data_val);

  samples0[sample] = timer.stop();
  timer.reset();

  correct = correct && stapl::is_null_reference(result0);

  // Case where element is 25% in to container
  set_known_value(x, quarter_idx);

  timer.reset();
  timer.start();

  auto result1 = stapl::find(x, data_val);

  samples25[sample] = timer.stop();

  correct = correct && stapl::index_of(result1) == quarter_idx;

  clear_known_value(x, quarter_idx);

  // Case where element is 75% in to container
  set_known_value(x, three_quarter_idx);

  timer.reset();
  timer.start();

  auto result2 = stapl::find(x, data_val);

  samples75[sample] = timer.stop();

  correct = correct && stapl::index_of(result2) == three_quarter_idx;

  clear_known_value(x, three_quarter_idx);
}


void test_find_if(view_t& x, std::vector<double>& samples0,
       std::vector<double>& samples25,
       std::vector<double>& samples75,
       bool& correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  counter_t timer;

  // Case where element is not found
  timer.reset();
  timer.start();

  auto result0 = stapl::find_if(x, equal_known(data_val));

  samples0[sample] = timer.stop();

  correct = correct && stapl::is_null_reference(result0);

  // Case where element is 25% in to container
  set_known_value(x, quarter_idx);

  timer.reset();
  timer.start();

  auto result1 = stapl::find_if(x, equal_known(data_val));

  samples25[sample] = timer.stop();

  correct = correct && stapl::index_of(result1) == quarter_idx;

  clear_known_value(x, quarter_idx);

  // Case where element is 75% in to container
  set_known_value(x, three_quarter_idx);

  timer.reset();
  timer.start();

  auto result2 = stapl::find_if(x, equal_known(data_val));

  samples75[sample] = timer.stop();

  correct = correct && stapl::index_of(result2) == three_quarter_idx;

  clear_known_value(x, three_quarter_idx);
}

void test_find_first_of(view_t& x, std::vector<double>& samples0,
       std::vector<double>& samples25, std::vector<double>& samples75,
       bool& correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  counter_t timer;

  // Declare second vector of values to be used in find_first_of call, default
  // constructing the elements and then setting the last element equal to the
  // known value.
  stapl::distribution_spec<> data_dist = stapl::balance(25);
  container_t search_space_cont(data_dist);
  view_t search_space(search_space_cont);
  search_space[24] = data_val;

  // ensure all locations have initialized search_space
  stapl::rmi_fence();

  // Case where element is not found
  timer.reset();
  timer.start();

  auto result0 = stapl::find_first_of(x, search_space);

  samples0[sample] = timer.stop();

  correct = correct && stapl::is_null_reference(result0);

  // Case where element is 25% in to container
  set_known_value(x, quarter_idx);

  timer.reset();
  timer.start();

  auto result1 = stapl::find_first_of(x, search_space);

  samples25[sample] = timer.stop();

  correct = correct && stapl::index_of(result1) == quarter_idx;

  clear_known_value(x, quarter_idx);

  // Case where element is 75% in to container
  set_known_value(x, three_quarter_idx);

  timer.reset();
  timer.start();

  auto result2 = stapl::find_first_of(x, search_space);

  samples75[sample] = timer.stop();

  correct = correct && stapl::index_of(result2) == three_quarter_idx;

  clear_known_value(x, three_quarter_idx);
}


void test_adjacent_find(view_t& x, std::vector<double>& samples0,
       std::vector<double>& samples25, std::vector<double>& samples75,
       bool& correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  counter_t timer;

  // Case where element is not found
  timer.reset();
  timer.start();

  auto result0 = stapl::adjacent_find(x);

  samples0[sample] = timer.stop();

  correct = correct && stapl::is_null_reference(result0);

  // Case where element is 25% in to container
  set_known_value(x, quarter_idx);
  set_known_value(x, quarter_idx+1);

  timer.reset();
  timer.start();

  auto result1 = stapl::adjacent_find(x);

  samples25[sample] = timer.stop();

  correct = correct && stapl::index_of(result1) == quarter_idx;

  clear_known_value(x, quarter_idx, 2);

  // Case where element is 75% in to container
  set_known_value(x, three_quarter_idx);
  set_known_value(x, three_quarter_idx+1);

  timer.reset();
  timer.start();

  auto result2 = stapl::adjacent_find(x);

  samples75[sample] = timer.stop();

  correct = correct && stapl::index_of(result2) == three_quarter_idx;

  clear_known_value(x, three_quarter_idx, 2);
}


void test_count(view_t& x, std::vector<double>& samples,
       bool& correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  set_known_value(x, quarter_idx);
  set_known_value(x, three_quarter_idx);

  counter_t timer;

  timer.reset();
  timer.start();

  auto result = stapl::count(x, data_val);

  samples[sample] = timer.stop();

  clear_known_value(x, quarter_idx);
  clear_known_value(x, three_quarter_idx);

  correct = correct && result == 2;
}


void test_count_if(view_t& x, std::vector<double>& samples,
       bool& correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  set_known_value(x, quarter_idx);
  set_known_value(x, three_quarter_idx);

  counter_t timer;

  timer.reset();
  timer.start();

  auto result = stapl::count_if(x, equal_known(data_val));

  samples[sample] = timer.stop();

  clear_known_value(x, quarter_idx);
  clear_known_value(x, three_quarter_idx);

  correct = correct && result == 2;
}


void test_mismatch_equal(view_t& x, view_t& y,
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
  stapl::copy(x, y);

  counter_t timer;

  // Evaluate algorithms when sequences are equal
  timer.reset();
  timer.start();

  bool equal_result = stapl::equal(x, y);

  equal_samples0[sample] = timer.stop();

  equal_correct = equal_correct && equal_result;

  timer.reset();
  timer.start();

  auto mismatch_result0 = stapl::mismatch(x, y);

  mismatch_samples0[sample] = timer.stop();

  mismatch_correct = mismatch_correct &&
    stapl::is_null_reference(mismatch_result0.first) &&
    stapl::is_null_reference(mismatch_result0.second);

  // Set mismatch 75% in to y
  set_known_value(y, three_quarter_idx);

  timer.reset();
  timer.start();

  equal_result = stapl::equal(x, y);

  equal_samples75[sample] = timer.stop();

  equal_correct = equal_correct && !equal_result;

  timer.reset();
  timer.start();

  auto mismatch_result1 = stapl::mismatch(x, y);

  mismatch_samples75[sample] = timer.stop();

  mismatch_correct = mismatch_correct &&
    stapl::index_of(mismatch_result1.first) == three_quarter_idx &&
    stapl::index_of(mismatch_result1.second) == three_quarter_idx;

  // Set mismatch 25% in to x
  set_known_value(x, quarter_idx);

  timer.reset();
  timer.start();

  equal_result = stapl::equal(x, y);

  equal_samples25[sample] = timer.stop();

  equal_correct = equal_correct && !equal_result;

  timer.reset();
  timer.start();

  auto mismatch_result2 = stapl::mismatch(x, y);

  mismatch_samples25[sample] = timer.stop();

  mismatch_correct = mismatch_correct &&
    stapl::index_of(mismatch_result2.first) == quarter_idx &&
    stapl::index_of(mismatch_result2.second) == quarter_idx;
}

void test_search_search_n(view_t& x,
       std::vector<double>& search_samples0,
       std::vector<double>& search_samples25,
       std::vector<double>& search_samples75,
       std::vector<double>& search_n_samples0,
       std::vector<double>& search_n_samples25,
       std::vector<double>& search_n_samples75,
       bool& search_correct, bool search_n_correct, int sample)
{
  size_t quarter_idx(0.25*x.size()), three_quarter_idx(0.75*x.size());

  counter_t timer;

  // Declare second vector of values to be used in the search call.
  stapl::distribution_spec<> data_dist = stapl::balance(25);
  container_t substring_cont(data_dist, data_val);
  view_t substring(substring_cont);

  // Case where element is not found
  timer.start();

  auto result0 = stapl::search(x, substring);

  search_samples0[sample] = timer.stop();

  search_correct = search_correct && stapl::is_null_reference(result0);

  timer.reset();
  timer.start();

  auto result_n0 = stapl::search_n(x, 25, data_val);

  search_n_samples0[sample] = timer.stop();

  search_n_correct = search_n_correct && stapl::is_null_reference(result_n0);

  // Case where element is 25% in to container
  for (int i = 0; i != 25; ++i)
    set_known_value(x, quarter_idx+i);

  timer.reset();
  timer.start();

  auto result1 = stapl::search(x, substring);

  search_samples25[sample] = timer.stop();

  search_correct = search_correct && stapl::index_of(result1) == quarter_idx;

  timer.reset();
  timer.start();

  auto result_n1 = stapl::search_n(x, 25, data_val);

  search_n_samples25[sample] = timer.stop();

  search_n_correct = search_n_correct &&
    stapl::index_of(result_n1) == quarter_idx;

  clear_known_value(x, quarter_idx, 25);

  // Case where element is 75% in to container
  for (int i = 0; i != 25; ++i)
    set_known_value(x, three_quarter_idx+i);

  timer.reset();
  timer.start();

  auto result2 = stapl::search(x, substring);

  search_samples75[sample] = timer.stop();

  search_correct = search_correct &&
    stapl::index_of(result2) == three_quarter_idx;

  timer.reset();
  timer.start();

  auto result_n2 = stapl::search_n(x, 25, data_val);

  search_n_samples75[sample] = timer.stop();

  search_n_correct = search_n_correct &&
    stapl::index_of(result_n2) == three_quarter_idx;

  clear_known_value(x, three_quarter_idx, 25);
}
