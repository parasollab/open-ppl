/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <boost/lexical_cast.hpp>
#include <stapl/array.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/views/system_view.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/runtime/counter/default_counters.hpp>
#include "../../confint.hpp"
#include "../shared/viewbased_dist_funcs.hpp"

typedef stapl::counter<stapl::default_timer> counter_t;

template <typename Container>
std::tuple<double, double, bool>
compute(Container& c, std::stringstream& o, bool default_value = false)
{
  stapl::array_view<Container> cv(c);

  // if array constructed with a default value, verify correctness of
  // constructor
  bool def_ctor = !default_value;
  long int zero = 0;

  if (default_value)
  {
    unsigned long int res = stapl::accumulate(cv, zero);
    def_ctor = res == cv.size()*99 ? true : false;
  }

  counter_t gen_timer, acc_timer;
  gen_timer.reset();
  acc_timer.reset();

  gen_timer.start();
  stapl::generate(cv, stapl::sequence<long int>(0,1));
  gen_timer.stop();

  acc_timer.start();
  long int res = stapl::accumulate(cv, zero);
  acc_timer.stop();

  // check that all locations received the same value.
  bool all_eq = all_locs_equal()(res);

  // check that the value received is correct.
  long int n = c.size();
  if (!def_ctor || !all_eq || res != (n-1)*n/2)
  {
    o << "Status: FAIL\n";
    o << "Note: Error in computation. Expected " << (n-1)*n/2
      << " Computed " << res << " All loc equal " << all_eq
      << "Default value constructor " << def_ctor << "\n";
    return std::make_tuple(gen_timer.value(), acc_timer.value(), false);
  }
  stapl::rmi_fence();
  return std::make_tuple(gen_timer.value(), acc_timer.value(), true);
}


std::tuple<double, double, bool>
run_balanced(long int n, counter_t& timer, std::stringstream& o)
{
  timer.start();
  stapl::array<long int> b(n);
  timer.stop();
  return compute(b, o);
}


typedef stapl::distribution_spec<> distribution_spec;

template <typename DistSpec>
std::tuple<double, double, bool>
run_vb(DistSpec& spec, counter_t& timer, std::stringstream& o,
       bool default_value = false)
{
  typedef DistSpec partitioning_view_type;
  if (!default_value)
  {
    timer.start();
    stapl::array<long int,
      stapl::view_based_partition<partitioning_view_type>,
      stapl::view_based_mapper<partitioning_view_type> > a(spec);
    timer.stop();
    return compute(a, o, default_value);
  }
  else
  {
    long int dv = 99;
    timer.start();
    stapl::array<long int,
      stapl::view_based_partition<partitioning_view_type>,
      stapl::view_based_mapper<partitioning_view_type> > a(spec, dv);
    timer.stop();
    return compute(a, o, default_value);
  }
}


template <typename PartInfoContainer>
std::tuple<double, double, bool>
run_cb(PartInfoContainer& part_info, counter_t& timer, std::stringstream& o,
       bool default_value = false)
{
  typedef stapl::distribution_spec<> partitioning_view_type;
  if (!default_value)
  {
    timer.start();
    stapl::array<long int,
      stapl::view_based_partition<partitioning_view_type, PartInfoContainer>,
      stapl::view_based_mapper<partitioning_view_type> > a(part_info);
    timer.stop();
    return compute(a, o, default_value);
  }
  else
  {
    long int dv = 99;
    timer.start();
    stapl::array<long int,
      stapl::view_based_partition<partitioning_view_type, PartInfoContainer>,
      stapl::view_based_mapper<partitioning_view_type> > a(part_info, dv);
    timer.stop();
    return compute(a, o, default_value);
  }
}


stapl::exit_code stapl_main(int argc, char** argv)
{
  long int n;
  long int blk_sz;
  int samples;
  if (argc > 3)
  {
    n = boost::lexical_cast<long int>(argv[1]);
    blk_sz = boost::lexical_cast<long int>(argv[2]);
    samples = boost::lexical_cast<long int>(argv[3]);
  }
  else
  {
    n = 1024;
    blk_sz = n / 64;
    samples = 1;
  }

  if (blk_sz == 0)
  {
    printf("n too small.  Must be at least %d\n",512*16);
    return EXIT_FAILURE;
  }

  // Controller for test of stapl::balanced partition
  counter_t bal_arr;
  confidence_interval_controller bal_arr_ctrl(samples, samples, 0.05);

  // Controller for tests of view-based distributions
  // Only one controller is used for simplicity.
  counter_t vb_arr;
  confidence_interval_controller bal_vb_arr_ctrl(samples, samples, 0.05);

  // Times for stapl::balanced partition
  std::vector<double> bal_gen_times;
  std::vector<double> bal_acc_times;
  std::stringstream bal_report;
  bal_report << "Test : array_default_dist\n"
             << "Version : STAPL\n";
  bool bal_pass = true;

  // Times for balanced distribution
  std::vector<double> bal_vb_gen_times;
  std::vector<double> bal_vb_acc_times;
  std::stringstream bal_vb_report;
  bal_vb_report << "Test : array_viewbased_balanced_dist\n"
                << "Version : STAPL\n";
  bool bal_vb_pass = true;

  // Times for block distribution
  std::vector<double> blk_vb_ctr_times;
  std::vector<double> blk_vb_gen_times;
  std::vector<double> blk_vb_acc_times;
  std::stringstream blk_vb_report;
  blk_vb_report << "Test : array_viewbased_blocked_dist\n"
                << "Version : STAPL\n";
  bool blk_vb_pass = true;

  // Times for block-cyclic distribution
  std::vector<double> blk_cyc_vb_ctr_times;
  std::vector<double> blk_cyc_vb_gen_times;
  std::vector<double> blk_cyc_vb_acc_times;
  std::stringstream blk_cyc_vb_report;
  blk_cyc_vb_report << "Test : array_viewbased_block_cyclic_dist\n"
                    << "Version : STAPL\n";
  bool blk_cyc_vb_pass = true;

  // Times for arbitrary distribution
  std::vector<double> arb_vb_ctr_times;
  std::vector<double> arb_vb_gen_times;
  std::vector<double> arb_vb_acc_times;
  std::stringstream arb_vb_report;
  arb_vb_report << "Test : array_viewbased_arbitrary_dist\n"
                << "Version : STAPL\n";
  bool arb_vb_pass = true;

  // Times for container-based arbitrary distribution
  std::vector<double> cb_arb_vb_ctr_times;
  std::vector<double> cb_arb_vb_gen_times;
  std::vector<double> cb_arb_vb_acc_times;
  std::stringstream cb_arb_vb_report;
  cb_arb_vb_report << "Test : array_viewbased_cb_arbitrary_dist\n"
                   << "Version : STAPL\n";
  bool cb_arb_vb_pass = true;


  stapl::location_type nlocs = stapl::get_num_locations();
  stapl::array<stapl::arbitrary_partition_info>
    part_info(stapl::get_num_locations());
  stapl::do_once([&](void) {
    unsigned long int denominator = 2;
    unsigned long int upper_bound = nlocs != 1 ? n / denominator : n;
    unsigned long int lower_bound = 0;
    if (nlocs != 1) {
      for (unsigned int i = 0; i != nlocs-1; ++i)
      {
        part_info[i] =
          stapl::arbitrary_partition_info(lower_bound, upper_bound-1, i);
        denominator *= 2;
        lower_bound = upper_bound;
        upper_bound += n / denominator;
      }
      lower_bound = part_info[nlocs-2].domain().second+1;
      upper_bound = lower_bound + (part_info[nlocs-2].domain().second -
                                       part_info[nlocs-2].domain().first) + 1;
    }
    part_info[nlocs-1] =
      stapl::arbitrary_partition_info(lower_bound, upper_bound-1, nlocs-1);
  });
  stapl::array_view<stapl::array<stapl::arbitrary_partition_info>>
    part_view(part_info);

  std::tuple<double, double, bool> alg_times;
  while (bal_arr_ctrl.iterate() || bal_vb_arr_ctrl.iterate())
  {
    // Test default distributed array
    bal_arr.reset();
    alg_times = run_balanced(n, bal_arr, bal_report);
    bal_arr_ctrl.push_back(bal_arr.value());
    bal_gen_times.push_back(std::get<0>(alg_times));
    bal_acc_times.push_back(std::get<1>(alg_times));
    bal_pass = bal_pass && std::get<2>(alg_times);

    // Test view-based balanced distribution
    distribution_spec bal_spec = stapl::balance(n);

    vb_arr.reset();
    // construction uses a default value, which requires traversing components.
    // This should result in slightly slower construction, but still faster than
    // the other constructors.
    alg_times = run_vb(bal_spec, vb_arr, bal_vb_report, true);
    bal_vb_arr_ctrl.push_back(vb_arr.value());
    bal_vb_gen_times.push_back(std::get<0>(alg_times));
    bal_vb_acc_times.push_back(std::get<1>(alg_times));
    bal_vb_pass = bal_vb_pass && std::get<2>(alg_times);

    // Test view-based blocked distribution
    distribution_spec blk_spec = stapl::block(n, blk_sz);

    vb_arr.reset();
    alg_times = run_vb(blk_spec, vb_arr, blk_vb_report);
    blk_vb_ctr_times.push_back(vb_arr.value());
    blk_vb_gen_times.push_back(std::get<0>(alg_times));
    blk_vb_acc_times.push_back(std::get<1>(alg_times));
    blk_vb_pass = blk_vb_pass && std::get<2>(alg_times);

    // Test view-based block-cyclic distribution
    distribution_spec blk_cyc_spec = stapl::block_cyclic(n, blk_sz);

    vb_arr.reset();
    alg_times = run_vb(blk_cyc_spec, vb_arr, blk_cyc_vb_report);
    blk_cyc_vb_ctr_times.push_back(vb_arr.value());
    blk_cyc_vb_gen_times.push_back(std::get<0>(alg_times));
    blk_cyc_vb_acc_times.push_back(std::get<1>(alg_times));
    blk_cyc_vb_pass = blk_cyc_vb_pass && std::get<2>(alg_times);

    // Test view-based arbitrary distribution
    uneven_blocks ueb;
    reverse_parts rp(stapl::get_num_locations(), stapl::get_num_locations());
    distribution_spec arb_spec = stapl::arbitrary(n, stapl::get_num_locations(),
                                                  ueb, rp);
    vb_arr.reset();
    alg_times = run_vb(arb_spec, vb_arr, arb_vb_report);
    arb_vb_ctr_times.push_back(vb_arr.value());
    arb_vb_gen_times.push_back(std::get<0>(alg_times));
    arb_vb_acc_times.push_back(std::get<1>(alg_times));
    arb_vb_pass = arb_vb_pass && std::get<2>(alg_times);

    // Test view-based arbitrary distribution that uses a container of partition
    // information.
    vb_arr.reset();
    alg_times = run_cb(part_view, vb_arr, cb_arb_vb_report);
    cb_arb_vb_ctr_times.push_back(vb_arr.value());
    cb_arb_vb_gen_times.push_back(std::get<0>(alg_times));
    cb_arb_vb_acc_times.push_back(std::get<1>(alg_times));
    cb_arb_vb_pass = cb_arb_vb_pass && std::get<2>(alg_times);
  }


  // Output results for default balanced array
  if (bal_pass)
    bal_report << "Status : PASS\n";
  else
    bal_report << "Status : FAIL\n";
  bal_arr_ctrl.report(bal_report);

  std::stringstream bal_arr_gen;
  bal_arr_gen << "Test : generate_on_bal_array\n"
              << "Version : STAPL\n";
  if (bal_pass)
    bal_arr_gen << "Status : PASS\n";
  else
    bal_arr_gen << "Status : FAIL\n";
  report(bal_arr_gen, bal_gen_times);

  std::stringstream bal_arr_acc;
  bal_arr_acc << "Test : accumulate_on_bal_array\n"
              << "Version : STAPL\n";
  if (bal_pass)
    bal_arr_acc << "Status : PASS\n";
  else
    bal_arr_acc << "Status : FAIL\n";
  report(bal_arr_acc, bal_acc_times);

  stapl::do_once([&](void) {
    std::cerr << bal_report.str();
    std::cerr << bal_arr_gen.str();
    std::cerr << bal_arr_acc.str();
  });


  // Output results for view-based balanced array
  if (bal_vb_pass)
    bal_vb_report << "Status : PASS\n";
  else
    bal_vb_report << "Status : FAIL\n";
  bal_vb_arr_ctrl.report(bal_vb_report);

  std::stringstream bal_vb_arr_gen;
  bal_vb_arr_gen << "Test : generate_on_vb_bal_array\n"
              << "Version : STAPL\n";
  if (bal_vb_pass)
    bal_vb_arr_gen << "Status : PASS\n";
  else
    bal_vb_arr_gen << "Status : FAIL\n";
  report(bal_vb_arr_gen, bal_vb_gen_times);

  std::stringstream bal_vb_arr_acc;
  bal_vb_arr_acc << "Test : accumulate_on_vb_bal_array\n"
              << "Version : STAPL\n";
  if (bal_vb_pass)
    bal_vb_arr_acc << "Status : PASS\n";
  else
    bal_vb_arr_acc << "Status : FAIL\n";
  report(bal_vb_arr_acc, bal_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << bal_vb_report.str();
    std::cerr << bal_vb_arr_gen.str();
    std::cerr << bal_vb_arr_acc.str();
  });


  // Output results for view-based block array
  std::stringstream blk_vb_arr_ctr;
  blk_vb_arr_ctr << "Test : construct_vb_blk_array\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_arr_ctr << "Status : PASS\n";
  else
    blk_vb_arr_ctr << "Status : FAIL\n";
  report(blk_vb_arr_ctr, blk_vb_ctr_times);

  std::stringstream blk_vb_arr_gen;
  blk_vb_arr_gen << "Test : generate_on_vb_blk_array\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_arr_gen << "Status : PASS\n";
  else
    blk_vb_arr_gen << "Status : FAIL\n";
  report(blk_vb_arr_gen, blk_vb_gen_times);

  std::stringstream blk_vb_arr_acc;
  blk_vb_arr_acc << "Test : accumulate_on_vb_blk_array\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_arr_acc << "Status : PASS\n";
  else
    blk_vb_arr_acc << "Status : FAIL\n";
  report(blk_vb_arr_acc, blk_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << blk_vb_arr_ctr.str();
    std::cerr << blk_vb_arr_gen.str();
    std::cerr << blk_vb_arr_acc.str();
  });


  // Output results for view-based block-cyclic array
  std::stringstream blk_cyc_vb_arr_ctr;
  blk_cyc_vb_arr_ctr << "Test : construct_vb_blk_cyc_array\n"
                     << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_arr_ctr << "Status : PASS\n";
  else
    blk_cyc_vb_arr_ctr << "Status : FAIL\n";
  report(blk_cyc_vb_arr_ctr, blk_cyc_vb_ctr_times);

  std::stringstream blk_cyc_vb_arr_gen;
  blk_cyc_vb_arr_gen << "Test : generate_on_vb_blk_cyc_array\n"
                     << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_arr_gen << "Status : PASS\n";
  else
    blk_cyc_vb_arr_gen << "Status : FAIL\n";
  report(blk_cyc_vb_arr_gen, blk_cyc_vb_gen_times);

  std::stringstream blk_cyc_vb_arr_acc;
  blk_cyc_vb_arr_acc << "Test : accumulate_on_vb_blk_cyc_array\n"
                     << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_arr_acc << "Status : PASS\n";
  else
    blk_cyc_vb_arr_acc << "Status : FAIL\n";
  report(blk_cyc_vb_arr_acc, blk_cyc_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << blk_cyc_vb_arr_ctr.str();
    std::cerr << blk_cyc_vb_arr_gen.str();
    std::cerr << blk_cyc_vb_arr_acc.str();
  });

  // Output results for view-based arbitrary array
  std::stringstream arb_vb_arr_ctr;
  arb_vb_arr_ctr << "Test : construct_vb_arb_array\n"
                 << "Version : STAPL\n";
  if (arb_vb_pass)
    arb_vb_arr_ctr << "Status : PASS\n";
  else
    arb_vb_arr_ctr << "Status : FAIL\n";
  report(arb_vb_arr_ctr, arb_vb_ctr_times);

  std::stringstream arb_vb_arr_gen;
  arb_vb_arr_gen << "Test : generate_on_vb_arb_array\n"
                 << "Version : STAPL\n";
  if (arb_vb_pass)
    arb_vb_arr_gen << "Status : PASS\n";
  else
    arb_vb_arr_gen << "Status : FAIL\n";
  report(arb_vb_arr_gen, arb_vb_gen_times);

  std::stringstream arb_vb_arr_acc;
  arb_vb_arr_acc << "Test : accumulate_on_vb_arb_array\n"
                 << "Version : STAPL\n";
  if (arb_vb_pass)
    arb_vb_arr_acc << "Status : PASS\n";
  else
    arb_vb_arr_acc << "Status : FAIL\n";
  report(arb_vb_arr_acc, arb_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << arb_vb_arr_ctr.str();
    std::cerr << arb_vb_arr_gen.str();
    std::cerr << arb_vb_arr_acc.str();
  });


  // Output results for view-based container-based arbitrary array
  std::stringstream cb_arb_vb_arr_ctr;
  cb_arb_vb_arr_ctr << "Test : construct_vb_cb_arb_array\n"
                    << "Version : STAPL\n";
  if (cb_arb_vb_pass)
    cb_arb_vb_arr_ctr << "Status : PASS\n";
  else
    cb_arb_vb_arr_ctr << "Status : FAIL\n";
  report(cb_arb_vb_arr_ctr, cb_arb_vb_ctr_times);

  std::stringstream cb_arb_vb_arr_gen;
  cb_arb_vb_arr_gen << "Test : generate_on_vb_cb_arb_array\n"
                    << "Version : STAPL\n";
  if (cb_arb_vb_pass)
    cb_arb_vb_arr_gen << "Status : PASS\n";
  else
    cb_arb_vb_arr_gen << "Status : FAIL\n";
  report(cb_arb_vb_arr_gen, cb_arb_vb_gen_times);

  std::stringstream cb_arb_vb_arr_acc;
  cb_arb_vb_arr_acc << "Test : accumulate_on_cb_vb_arb_array\n"
                    << "Version : STAPL\n";
  if (cb_arb_vb_pass)
    cb_arb_vb_arr_acc << "Status : PASS\n";
  else
    cb_arb_vb_arr_acc << "Status : FAIL\n";
  report(cb_arb_vb_arr_acc, cb_arb_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << cb_arb_vb_arr_ctr.str();
    std::cerr << cb_arb_vb_arr_gen.str();
    std::cerr << cb_arb_vb_arr_acc.str();
  });

  return EXIT_SUCCESS;
}

