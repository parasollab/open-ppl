/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <boost/lexical_cast.hpp>
#include <stapl/list.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/views/system_view.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/numeric.hpp>
#include <stapl/runtime/counter/default_counters.hpp>
#include "../../confint.hpp"

typedef stapl::counter<stapl::default_timer> counter_t;

struct all_locs_equal
  : public stapl::p_object
{
private:
  long int m_value;

public:
  long int get_value(void)
  { return m_value; }

  bool operator()(long int value)
  {
    m_value = value;
    stapl::future<long int> r =
      stapl::allreduce_rmi(stapl::plus<long int>(),
                           this->get_rmi_handle(),
                           &all_locs_equal::get_value);
    long int result = r.get();
    return result == value*this->get_num_locations() ? true : false;
  }
};


template <typename Container>
std::tuple<double, double, bool>
compute(Container& c, std::stringstream& o, bool default_value = false)
{
  stapl::list_view<Container> cv(c);

  // if list constructed with a default value, verify correctness of
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
  stapl::list<long int> b(n);
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
    stapl::list<long int,
      stapl::view_based_partition<partitioning_view_type>,
      stapl::view_based_mapper<partitioning_view_type> > a(spec);
    timer.stop();
    return compute(a, o, default_value);
  }
  else
  {
    long int dv = 99;
    timer.start();
    stapl::list<long int,
      stapl::view_based_partition<partitioning_view_type>,
      stapl::view_based_mapper<partitioning_view_type> > a(spec, dv);
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
  bal_report << "Test : list_default_dist\n"
             << "Version : STAPL\n";
  bool bal_pass = true;

  // Times for balanced distribution
  std::vector<double> bal_vb_gen_times;
  std::vector<double> bal_vb_acc_times;
  std::stringstream bal_vb_report;
  bal_vb_report << "Test : list_viewbased_balanced_dist\n"
                << "Version : STAPL\n";
  bool bal_vb_pass = true;

  // Times for block distribution
  std::vector<double> blk_vb_ctr_times;
  std::vector<double> blk_vb_gen_times;
  std::vector<double> blk_vb_acc_times;
  std::stringstream blk_vb_report;
  blk_vb_report << "Test : list_viewbased_blocked_dist\n"
                << "Version : STAPL\n";
  bool blk_vb_pass = true;

  // Times for block-cyclic distribution
  std::vector<double> blk_cyc_vb_ctr_times;
  std::vector<double> blk_cyc_vb_gen_times;
  std::vector<double> blk_cyc_vb_acc_times;
  std::stringstream blk_cyc_vb_report;
  blk_cyc_vb_report << "Test : list_viewbased_block_cyclic_dist\n"
                    << "Version : STAPL\n";
  bool blk_cyc_vb_pass = true;

  std::tuple<double, double, bool> alg_times;
  while (bal_arr_ctrl.iterate() || bal_vb_arr_ctrl.iterate())
  {
    // Test default distributed list
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
    distribution_spec blk_cyc_spec =
      stapl::block_cyclic(n, blk_sz);

    vb_arr.reset();
    alg_times = run_vb(blk_cyc_spec, vb_arr, blk_cyc_vb_report);
    blk_cyc_vb_ctr_times.push_back(vb_arr.value());
    blk_cyc_vb_gen_times.push_back(std::get<0>(alg_times));
    blk_cyc_vb_acc_times.push_back(std::get<1>(alg_times));
    blk_cyc_vb_pass = blk_cyc_vb_pass && std::get<2>(alg_times);
  }


  // Output results for default balanced list
  if (bal_pass)
    bal_report << "Status : PASS\n";
  bal_arr_ctrl.report(bal_report);

  std::stringstream bal_arr_gen;
  bal_arr_gen << "Test : generate_on_bal_list\n"
              << "Version : STAPL\n";
  if (bal_pass)
    bal_arr_gen << "Status : PASS\n";
  report(bal_arr_gen, bal_gen_times);

  std::stringstream bal_arr_acc;
  bal_arr_acc << "Test : accumulate_on_bal_list\n"
              << "Version : STAPL\n";
  if (bal_pass)
    bal_arr_acc << "Status : PASS\n";
  report(bal_arr_acc, bal_acc_times);

  stapl::do_once([&](void) {
    std::cerr << bal_report.str();
    std::cerr << bal_arr_gen.str();
    std::cerr << bal_arr_acc.str();
  });


  // Output results for view-based balanced list
  if (bal_vb_pass)
    bal_vb_report << "Status : PASS\n";
  bal_vb_arr_ctrl.report(bal_vb_report);

  std::stringstream bal_vb_arr_gen;
  bal_vb_arr_gen << "Test : generate_on_vb_bal_list\n"
              << "Version : STAPL\n";
  if (bal_vb_pass)
    bal_vb_arr_gen << "Status : PASS\n";
  report(bal_vb_arr_gen, bal_vb_gen_times);

  std::stringstream bal_vb_arr_acc;
  bal_vb_arr_acc << "Test : accumulate_on_vb_bal_list\n"
              << "Version : STAPL\n";
  if (bal_vb_pass)
    bal_vb_arr_acc << "Status : PASS\n";
  report(bal_vb_arr_acc, bal_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << bal_vb_report.str();
    std::cerr << bal_vb_arr_gen.str();
    std::cerr << bal_vb_arr_acc.str();
  });

  // Output results for view-based block list
  std::stringstream blk_vb_arr_ctr;
  blk_vb_arr_ctr << "Test : construct_vb_blk_list\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_report << "Status : PASS\n";
  report(blk_vb_arr_ctr, blk_vb_ctr_times);

  std::stringstream blk_vb_arr_gen;
  blk_vb_arr_gen << "Test : generate_on_vb_blk_list\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_arr_gen << "Status : PASS\n";
  report(blk_vb_arr_gen, blk_vb_gen_times);

  std::stringstream blk_vb_arr_acc;
  blk_vb_arr_acc << "Test : accumulate_on_vb_blk_list\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_arr_acc << "Status : PASS\n";
  report(blk_vb_arr_acc, blk_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << blk_vb_arr_ctr.str();
    std::cerr << blk_vb_arr_gen.str();
    std::cerr << blk_vb_arr_acc.str();
  });

  // Output results for view-based block-cyclic list
  std::stringstream blk_cyc_vb_arr_ctr;
  blk_cyc_vb_arr_ctr << "Test : construct_vb_blk_cyc_list\n"
              << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_report << "Status : PASS\n";
  report(blk_cyc_vb_arr_ctr, blk_cyc_vb_ctr_times);

  std::stringstream blk_cyc_vb_arr_gen;
  blk_cyc_vb_arr_gen << "Test : generate_on_vb_blk_cyc_list\n"
              << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_arr_gen << "Status : PASS\n";
  report(blk_cyc_vb_arr_gen, blk_cyc_vb_gen_times);

  std::stringstream blk_cyc_vb_arr_acc;
  blk_cyc_vb_arr_acc << "Test : accumulate_on_vb_blk_cyc_list\n"
              << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_arr_acc << "Status : PASS\n";
  report(blk_cyc_vb_arr_acc, blk_cyc_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << blk_cyc_vb_arr_ctr.str();
    std::cerr << blk_cyc_vb_arr_gen.str();
    std::cerr << blk_cyc_vb_arr_acc.str();
  });
  return EXIT_SUCCESS;
}
