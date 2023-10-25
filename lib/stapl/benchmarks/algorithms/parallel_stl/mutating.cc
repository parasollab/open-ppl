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
#include <sstream>
#include <functional>

int main(int argc, char *argv[])
{
  size_t size = 13;
  int num_threads = 1;
  double duplicate_ratio = 0.001;

  if (argc > 3)
  {
    size = atoll(argv[1]);
    num_threads = atoi( argv[2] );
    duplicate_ratio = atof(argv[3]);
  }
  else if (argc > 2)
  {
    size = atoll(argv[1]);
    num_threads = atoi( argv[2] );
  }
  else if (argc > 1)
    size = atoll(argv[1]);

  // Force the use of parallel STL versions of the algorithms
  // when using more than 1 thread.
  __gnu_parallel::_Settings s;
  if (num_threads > 1)
  {
    s.algorithm_strategy = __gnu_parallel::force_parallel;
    __gnu_parallel::_Settings::set(s);
  }

  omp_set_num_threads(num_threads);

  std::vector<double> gen_samples(10,0.), gen_n_samples(10,0.),
    shu_samples(10,0.), rep_samples(10,0.), rif_samples(10,0.),
    tra_samples(10,0.), mer_samples(10,0.), uni_samples(10,0.);

  bool gen_correct(true), gen_n_correct(true), shu_correct(true),
    rep_correct(true), rif_correct(true), tra_correct(true),
    mer_correct(true), uni_correct(true);

  // GENERATE
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);

    auto time = start_timer();

     __gnu_parallel::generate(x.begin(), x.end(), []{ return 81372.0; });

    gen_samples[sample] = stop_timer(time);

    gen_correct  = gen_correct && (x[std::rand()%size] == 81372.0);
  }
  report_result("b_generate","gnu_parallel",gen_correct, gen_samples);

  // GENERATE N
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);

    auto time = start_timer();

     __gnu_parallel::generate_n(x.begin(), size, []{ return 81372.0; });

    gen_n_samples[sample] = stop_timer(time);

    gen_n_correct  = gen_correct && (x[std::rand()%size] == 81372.0);
  }
  report_result("b_generate_n","gnu_parallel",gen_n_correct, gen_n_samples);

  // REPLACE IF and REPLACE
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);
    fill_random(x);

    auto time = start_timer();

    // Replace if less than 0.25
    __gnu_parallel::replace_if(x.begin(), x.end(),
      std::bind(std::less<double>(),std::placeholders::_1, 0.25), 0.5,
                               __gnu_parallel::parallel_balanced);

    rif_samples[sample] = stop_timer(time);

    rif_correct = rif_correct &&
      (std::find_if(x.begin(), x.end(), std::bind(std::less<double>(),
        std::placeholders::_1, 0.25)) == x.end());

    time = start_timer();

    // Replace 0 with 0.125
    __gnu_parallel::replace(x.begin(), x.end(), 0.5, 0.125,
                            __gnu_parallel::parallel_balanced);

    rep_samples[sample] = stop_timer(time);

    auto result = std::accumulate(x.begin(), x.end(), 0.);

    rep_correct = rep_correct && result/size > 0.49 && result/size < 0.51;
  }
  report_result("b_replace","gnu_parallel", rep_correct, rep_samples);
  report_result("b_replace_if","gnu_parallel", rif_correct, rif_samples);

  // TRANSFORM
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);
    fill_random(x);
    std::vector<data_t> original(x);

    auto time = start_timer();

    __gnu_parallel::transform(x.begin(), x.end(),  x.begin(),
      [](double i) { return i+1;}); // Increase by 1

    tra_samples[sample] = stop_timer(time);

    tra_correct = tra_correct && std::equal(x.begin(), x.end(),
      original.begin(), [](double x, double y) {return x == (y+1);});
  }
  report_result("b_transform","gnu_parallel", tra_correct, tra_samples);


  // UNIQUE COPY
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x1(size);
    fill_random(x1, duplicate_ratio);
    std::vector<data_t> x2(size);

    // unique_copy requires sorted input
    std::sort(x1.begin(), x1.end());

    double pre_copy = std::accumulate(x2.begin(), x2.end(), 0.0);
    auto time = start_timer();

    __gnu_parallel::unique_copy(x1.begin(), x1.end(), x2.begin());

    uni_samples[sample] = stop_timer(time);

    double post_copy = std::accumulate(x2.begin(), x2.end(), 0.0);
    uni_correct = uni_correct && (pre_copy != post_copy);
  }
  std::stringstream unique_copy_name;
  unique_copy_name << "b_unique_copy-" << duplicate_ratio;

  report_result(unique_copy_name.str().c_str(),"gnu_parallel",
                uni_correct, uni_samples);


  // NOTE: Our experiments show that the parallel implementation of
  //       random_shuffle and merge are faster sequentially than the
  //       implementation selected by default.
  if (num_threads == 1)
  {
    // Force the use of parallel STL versions of the algorithms 
    s.algorithm_strategy = __gnu_parallel::force_parallel;
    __gnu_parallel::_Settings::set(s);
  }

  // RANDOM_SHUFFLE
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x(size);
    fill_random(x);

    double initial = std::accumulate(x.begin(), x.end(), 0.0);
    auto time = start_timer();

    __gnu_parallel::random_shuffle(x.begin(), x.end());

    shu_samples[sample] = stop_timer(time);

    double post_shuffle = std::accumulate(x.begin(), x.end(), 0.0);
    // Checked with some room for error due to floating point arithmetic
    if ((post_shuffle * 1.05) <= initial || (post_shuffle * 0.95) >= initial )
      shu_correct = false;

  }
  report_result("b_random_shuffle","gnu_parallel",shu_correct, shu_samples);

  // MERGE
  for (int sample = 0; sample != 10; ++sample)
  {
    std::vector<data_t> x1(size);
    fill_random(x1);
    std::vector<data_t> x2(size);
    fill_random(x2);
    std::vector<data_t> x3(size*2);

    // merge requires sorted inputs
    std::sort(x1.begin(), x1.end());
    std::sort(x2.begin(), x2.end());

    double pre_merge =  std::accumulate(x1.begin(), x1.end(), 0.0);
    pre_merge += std::accumulate(x2.begin(), x2.end(), 0.0);

    auto time = start_timer();

    __gnu_parallel::merge(x1.begin(), x1.end(), x2.begin(), x2.end(),
      x3.begin());

    mer_samples[sample] = stop_timer(time);

    double post_merge = std::accumulate(x3.begin(), x3.end(), 0.0);

    if ((post_merge * 1.05) <= pre_merge || (post_merge * 0.95) >= pre_merge )
      mer_correct = false;
  }
  report_result("b_merge","gnu_parallel",mer_correct, mer_samples);

  return 0;
}
