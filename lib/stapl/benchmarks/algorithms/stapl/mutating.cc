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
#include <sstream>
#include <stapl/array.hpp>
#include <stapl/algorithms/functional.hpp>

template <typename T1, typename T2>
struct equal_plus_one
{
  typedef bool result_type;

  equal_plus_one()
  {
  }

  result_type operator() (const T1& x, const T2& y) const
  {
    return x == y+1;
  }
};

stapl::exit_code stapl_main(int argc, char *argv[])
{
  // Default values. Size is the number of elements in the tested arrays.
  // Duplicate Ratio is used in the data generation of unique copy to specify
  // the number of duplicates in the input.
  // Weak_scaling weak_scaling specifies whether the data should be generated
  // sequentially to ensure the same input is used for all core counts in a
  // strong scaling study, or in parallel which allows for faster generation
  // needed in weak scaling where the input grows with core count.
  size_t size = atoll(argv[1]);
  double duplicate_ratio = 0.001;
  int weak_scaling = 0; // True or false.
  int data_distribution = 0; // 0=balanced, 1=block, 2=block_cyclic
  stapl::distribution_spec<> data_dist = stapl::balance(size);
  stapl::distribution_spec<> data_distx2 = stapl::balance(size*2);
  int block_size=2048;

  if (argc > 5)
  {
    size = atoll(argv[1]);
    duplicate_ratio = atof(argv[2]);
    weak_scaling = atoi(argv[3]);
    data_distribution = atoi(argv[4]);
    block_size = atoi(argv[5]);
  }
  else if (argc > 4)
  {
    size = atoll(argv[1]);
    duplicate_ratio = atof(argv[2]);
    data_distribution = atoi(argv[3]);
    block_size = atoi(argv[4]);
  }
  else if (argc > 3)
  {
    size = atoll(argv[1]);
    data_distribution = atoi(argv[2]);
    block_size = atoi(argv[3]);
  }

  if (data_distribution == 1)
  {
    data_dist = stapl::block(size,block_size);
    data_distx2 = stapl::block(size*2,block_size);
  }
  else if (data_distribution == 2)
  {
    data_dist = stapl::block_cyclic(size,block_size);
    data_distx2 = stapl::block_cyclic(size*2,block_size);
  }

  std::vector<double> gen_samples(10,0.), gen_n_samples(10,0.),
    shu_samples(10,0.), rep_samples(10,0.), rif_samples(10,0.),
    tra_samples(10,0.), mer_samples(10,0.), uni_samples(10,0.);

  bool gen_correct(true), gen_n_correct(true), shu_correct(true),
    rep_correct(true), rif_correct(true), tra_correct(true),
    mer_correct(true), uni_correct(true);

  counter_t timer;

  // GENERATE
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont(data_dist);
    array_type_vw x(xcont);

    timer.reset();
    timer.start();

    stapl::generate(x, []{ return 81372.0; });

    gen_samples[sample] = timer.stop();

    stapl::do_once([&xcont, &gen_correct, &size]{
     gen_correct  = gen_correct && (xcont[std::rand()%size] == 81372.0);
    });
  }
  report_result("b_generate","STAPL",gen_correct, gen_samples);

  // GENERATE N
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont(data_dist);
    array_type_vw x(xcont);

    timer.reset();
    timer.start();

    stapl::generate_n(x, 0, size, []{ return 81372.0; });

    gen_n_samples[sample] = timer.stop();

    stapl::do_once([&xcont, &gen_n_correct, &size]{
      gen_n_correct  = gen_n_correct && (xcont[std::rand()%size] == 81372.0);
    });
  }
  report_result("b_generate_n","STAPL",gen_n_correct, gen_n_samples);

  // REPLACE IF and REPLACE
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont(data_dist);
    array_type_vw x(xcont);

    fill_random(x, weak_scaling);
    timer.reset();
    timer.start();

    stapl::replace_if(x, std::bind(std::less<double>(),
      std::placeholders::_1, 0.25), 0.5);

    rif_samples[sample] = timer.stop();

    auto result0 = stapl::find_if(x, std::bind(std::less<double>(),
      std::placeholders::_1, 0.25));

    rif_correct = rif_correct && stapl::is_null_reference(result0);

    timer.reset();
    timer.start();

    stapl::replace(x, 0.5, 0.125);

    rep_samples[sample] = timer.stop();

    auto result1 = stapl::accumulate(x, 0.);

    rep_correct = rep_correct && result1/size > 0.49 && result1/size < 0.51;
  }
  report_result("b_replace","STAPL", rep_correct, rep_samples);
  report_result("b_replace_if","STAPL", rif_correct, rif_samples);

  // TRANSFORM
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont(data_dist);
    array_type_vw x(xcont);

    fill_random(x, weak_scaling);
    array_type original(xcont);
    array_type_vw o(original);
    timer.reset();
    timer.start();

    stapl::transform(x, x, [](double i) { return i+1;});

    tra_samples[sample] = timer.stop();

    bool test = stapl::equal(x, o, equal_plus_one<double, double>());
    tra_correct = tra_correct && test;
   }
  report_result("b_transform","STAPL", tra_correct, tra_samples);

  // MERGE
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont1(data_dist);
    array_type_vw x1(xcont1);
    array_type xcont2(data_dist);
    array_type_vw x2(xcont2);
    array_type xcont3(data_distx2);
    array_type_vw x3(xcont3);

    fill_random(x1, weak_scaling);
    fill_random(x2, weak_scaling);

    // merge requires sorted inputs
    stapl::sort(x1);
    stapl::sort(x2);

    double post_merge = stapl::accumulate(x1,0.0);
    post_merge += stapl::accumulate(x2,0.0);

    timer.reset();
    timer.start();

    stapl::merge(x1, x2, x3);

    mer_samples[sample] = timer.stop();

    double pre_merge = stapl::accumulate(x3,0.0);
    if ((post_merge * 1.05) <= pre_merge || (post_merge * 0.95) >= pre_merge )
      mer_correct = false;
  }
  report_result("b_merge","STAPL",mer_correct, mer_samples);

  // UNIQUE COPY
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont1(data_dist);
    array_type_vw x1(xcont1);
    array_type xcont2(data_dist);
    array_type_vw x2(xcont2);

    // Generate block of duplicates for experiment
    fill_random(x1, weak_scaling, duplicate_ratio, false);

    // unique_copy requires sorted input
    stapl::sort(x1);

    double pre_copy = stapl::accumulate(x2,0);
    timer.reset();
    timer.start();

    stapl::unique_copy(x1, x2);

    uni_samples[sample] = timer.stop();

    double post_copy = stapl::accumulate(x2,0);
    uni_correct = uni_correct && (pre_copy != post_copy);
  }
  std::stringstream unique_copy_name;
  unique_copy_name << "b_unique_copy-" << duplicate_ratio;

  report_result(unique_copy_name.str().c_str(),"STAPL",
                uni_correct, uni_samples);

  // RANDOM SHUFFLE
  for (int sample = 0; sample != 10; ++sample)
  {
    array_type xcont(data_dist);
    array_type_vw x(xcont);

    fill_random(x, weak_scaling);
    double initial = stapl::accumulate(x,0.0);
    timer.reset();
    timer.start();

    stapl::random_shuffle(x);

    shu_samples[sample] = timer.stop();

    double post_shuffle = stapl::accumulate(x,0.0);
    // Checked with some room for error due to decimals not being exact.
    if ((post_shuffle * 1.05) <= initial || (post_shuffle * 0.95) >= initial )
      shu_correct = false;
  }
  report_result("b_random_shuffle","STAPL",shu_correct, shu_samples);

  return EXIT_SUCCESS;
}
