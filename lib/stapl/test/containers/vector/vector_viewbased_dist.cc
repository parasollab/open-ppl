/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <boost/lexical_cast.hpp>
#include <stapl/vector.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/views/system_view.hpp>
#include <stapl/runtime/counter/default_counters.hpp>
#include <stapl/utility/do_once.hpp>
#include "../../confint.hpp"
#include "../shared/viewbased_dist_funcs.hpp"

typedef stapl::counter<stapl::default_timer> counter_t;

template <typename Container>
std::tuple<double, double, bool>
compute(Container& c, std::stringstream& o, bool default_value = false)
{
  stapl::vector_view<Container> cv(c);

  // if vector constructed with a default value, verify correctness of
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
  return std::make_tuple(gen_timer.value(), acc_timer.value(), true);
}


template <typename Container>
std::tuple<double, double, bool>
compute_composed(Container& c, std::stringstream& o, unsigned int n1,
                 unsigned int n2, unsigned int n3)
{
  stapl::vector_view<Container> cv(c);

  counter_t gen_timer, acc_timer;
  gen_timer.reset();
  acc_timer.reset();

  gen_timer.start();
  stapl::map_func(outer_init(n1, n2, n3), cv,
                  stapl::counting_view<long int>(n1));
  gen_timer.stop();

  acc_timer.start();
  long int res = stapl::map_reduce(outer_sum(), stapl::plus<long int>(), cv);
  acc_timer.stop();

  // check that all locations received the same value.
  bool all_eq = all_locs_equal()(res);

  // check that the value received is correct.
  if (!all_eq || res != (n1*n2*n3-1)*n1*n2*n3/2)
  {
    o << "Status: FAIL\n";
    o << "Note: Error in computation. Expected " << (n1*n2*n3-1)*n1*n2*n3/2
      << " Computed " << res << " All loc equal " << all_eq << "\n";
    return std::make_tuple(gen_timer.value(), acc_timer.value(), false);
  }
  return std::make_tuple(gen_timer.value(), acc_timer.value(), true);
}


std::tuple<double, double, bool>
run_balanced(long int n, counter_t& timer, std::stringstream& o)
{
  timer.start();
  stapl::vector<long int> b(n);
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
    stapl::vector<long int,
      stapl::view_based_partition<partitioning_view_type>,
      stapl::view_based_mapper<partitioning_view_type> > a(spec);
    timer.stop();
    return compute(a, o, default_value);
  }
  else
  {
    long int dv = 99;
    timer.start();
    stapl::vector<long int,
      stapl::view_based_partition<partitioning_view_type>,
      stapl::view_based_mapper<partitioning_view_type> > a(spec, dv);
    timer.stop();
    return compute(a, o, default_value);
  }
}


template <typename DistSpec>
std::tuple<double, double, bool>
run_vb_composed(std::vector<DistSpec>& spec, counter_t& timer,
                std::stringstream& o)
{
  typedef DistSpec dist_spec;
  typedef stapl::view_based_partition<dist_spec> vb_part;
  typedef stapl::view_based_mapper<dist_spec> vb_map;
  typedef stapl::vector<
            stapl::vector<stapl::vector<long int, vb_part, vb_map>,
              vb_part, vb_map>,
            vb_part, vb_map> container;

  timer.start();
  container a(spec);
  timer.stop();
  return compute_composed(a, o, spec[0].domain().size(),
                          spec[1].domain().size(), spec[2].domain().size());
}


stapl::exit_code stapl_main(int argc, char** argv)
{
  long int n, n1, n2, n3;
  long int blk_sz, comp_blk_sz;
  int samples;
  if (argc > 7)
  {
    n = boost::lexical_cast<long int>(argv[1]);
    n1 = boost::lexical_cast<long int>(argv[2]);
    n2 = boost::lexical_cast<long int>(argv[3]);
    n3 = boost::lexical_cast<long int>(argv[4]);
    blk_sz = boost::lexical_cast<long int>(argv[5]);
    comp_blk_sz = boost::lexical_cast<long int>(argv[6]);
    samples = boost::lexical_cast<long int>(argv[7]);
  }
  else
  {
    n = 1024;
    n1 = 16;
    n2 = 16;
    n3 = 16;
    blk_sz = n / 64;
    comp_blk_sz = n3 / 8;
    samples = 1;
  }

  if (blk_sz == 0)
  {
    printf("n too small.  Must be at least %d\n",512*16);
    return EXIT_FAILURE;
  }

  // Controller for test of stapl::balanced partition
  counter_t bal_arr;
  confidence_interval_controller bal_vec_ctrl(samples, samples, 0.05);

  // Controller for tests of view-based distributions
  // Only one controller is used for simplicity.
  counter_t vb_arr;
  confidence_interval_controller bal_vb_vec_ctrl(samples, samples, 0.05);

  // Times for stapl::balanced partition
  std::vector<double> bal_gen_times;
  std::vector<double> bal_acc_times;
  std::stringstream bal_report;
  bal_report << "Test : vector_default_dist\n"
             << "Version : STAPL\n";
  bool bal_pass = true;

  // Times for balanced distribution
  std::vector<double> bal_vb_gen_times;
  std::vector<double> bal_vb_acc_times;
  std::stringstream bal_vb_report;
  bal_vb_report << "Test : vector_viewbased_balanced_dist\n"
                << "Version : STAPL\n";
  bool bal_vb_pass = true;

  // Times for block distribution
  std::vector<double> blk_vb_ctr_times;
  std::vector<double> blk_vb_gen_times;
  std::vector<double> blk_vb_acc_times;
  std::stringstream blk_vb_report;
  blk_vb_report << "Test : vector_viewbased_blocked_dist\n"
                << "Version : STAPL\n";
  bool blk_vb_pass = true;

  // Times for block-cyclic distribution
  std::vector<double> blk_cyc_vb_ctr_times;
  std::vector<double> blk_cyc_vb_gen_times;
  std::vector<double> blk_cyc_vb_acc_times;
  std::stringstream blk_cyc_vb_report;
  blk_cyc_vb_report << "Test : vector_viewbased_block_cyclic_dist\n"
                    << "Version : STAPL\n";
  bool blk_cyc_vb_pass = true;

  // Times for arbitrary distribution
  std::vector<double> arb_vb_ctr_times;
  std::vector<double> arb_vb_gen_times;
  std::vector<double> arb_vb_acc_times;
  std::stringstream arb_vb_report;
  arb_vb_report << "Test : vector_viewbased_arbitrary_dist\n"
                << "Version : STAPL\n";
  bool arb_vb_pass = true;

  // Times for composed cyclic/balanced/block-cyclic distribution
  std::vector<double> comp_vb_ctr_times;
  std::vector<double> comp_vb_gen_times;
  std::vector<double> comp_vb_acc_times;
  std::stringstream   comp_vb_report;
  comp_vb_report << "Test : homogeneous_comp_vector_dist\n"
                 << "Version : STAPL\n";
  bool comp_vb_pass = true;

  std::tuple<double, double, bool> alg_times;
  while (bal_vec_ctrl.iterate() || bal_vb_vec_ctrl.iterate())
  {
    // Test default distributed vector
    bal_arr.reset();
    alg_times = run_balanced(n, bal_arr, bal_report);
    bal_vec_ctrl.push_back(bal_arr.value());
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
    bal_vb_vec_ctrl.push_back(vb_arr.value());
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

    // Test homogeneous by level view-based distribution
    std::vector<distribution_spec> dist_specs =
      { stapl::cyclic(n1), stapl::balance(n2, stapl::current_level),
        stapl::block_cyclic(n3, comp_blk_sz, stapl::lowest_level) };
    vb_arr.reset();
    alg_times = run_vb_composed(dist_specs, vb_arr,
                                comp_vb_report);
    comp_vb_ctr_times.push_back(vb_arr.value());
    comp_vb_gen_times.push_back(std::get<0>(alg_times));
    comp_vb_acc_times.push_back(std::get<1>(alg_times));
    comp_vb_pass = comp_vb_pass && std::get<2>(alg_times);
  }


  // Output results for default balanced vector
  if (bal_pass)
    bal_report << "Status : PASS\n";
  else
    bal_report << "Status : FAIL\n";
  bal_vec_ctrl.report(bal_report);

  std::stringstream bal_vec_gen;
  bal_vec_gen << "Test : generate_on_bal_vector\n"
              << "Version : STAPL\n";
  if (bal_pass)
    bal_vec_gen << "Status : PASS\n";
  else
    bal_vec_gen << "Status : FAIL\n";
  report(bal_vec_gen, bal_gen_times);

  std::stringstream bal_vec_acc;
  bal_vec_acc << "Test : accumulate_on_bal_vector\n"
              << "Version : STAPL\n";
  if (bal_pass)
    bal_vec_acc << "Status : PASS\n";
  else
    bal_vec_acc << "Status : FAIL\n";
  report(bal_vec_acc, bal_acc_times);

  stapl::do_once([&](void) {
    std::cerr << bal_report.str();
    std::cerr << bal_vec_gen.str();
    std::cerr << bal_vec_acc.str();
  });


  // Output results for view-based balanced vector
  if (bal_vb_pass)
    bal_vb_report << "Status : PASS\n";
  else
    bal_vb_report << "Status : FAIL\n";
  bal_vb_vec_ctrl.report(bal_vb_report);

  std::stringstream bal_vb_vec_gen;
  bal_vb_vec_gen << "Test : generate_on_vb_bal_vector\n"
              << "Version : STAPL\n";
  if (bal_vb_pass)
    bal_vb_vec_gen << "Status : PASS\n";
  else
    bal_vb_vec_gen << "Status : FAIL\n";
  report(bal_vb_vec_gen, bal_vb_gen_times);

  std::stringstream bal_vb_vec_acc;
  bal_vb_vec_acc << "Test : accumulate_on_vb_bal_vector\n"
              << "Version : STAPL\n";
  if (bal_vb_pass)
    bal_vb_vec_acc << "Status : PASS\n";
  else
    bal_vb_vec_acc << "Status : FAIL\n";
  report(bal_vb_vec_acc, bal_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << bal_vb_report.str();
    std::cerr << bal_vb_vec_gen.str();
    std::cerr << bal_vb_vec_acc.str();
  });


  // Output results for view-based block vector
  std::stringstream blk_vb_vec_ctr;
  blk_vb_vec_ctr << "Test : construct_vb_blk_vector\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_vec_ctr << "Status : PASS\n";
  else
    blk_vb_vec_ctr << "Status : FAIL\n";
  report(blk_vb_vec_ctr, blk_vb_ctr_times);

  std::stringstream blk_vb_vec_gen;
  blk_vb_vec_gen << "Test : generate_on_vb_blk_vector\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_vec_gen << "Status : PASS\n";
  else
    blk_vb_vec_gen << "Status : FAIL\n";
  report(blk_vb_vec_gen, blk_vb_gen_times);

  std::stringstream blk_vb_vec_acc;
  blk_vb_vec_acc << "Test : accumulate_on_vb_blk_vector\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_vec_acc << "Status : PASS\n";
  else
    blk_vb_vec_acc << "Status : FAIL\n";
  report(blk_vb_vec_acc, blk_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << blk_vb_vec_ctr.str();
    std::cerr << blk_vb_vec_gen.str();
    std::cerr << blk_vb_vec_acc.str();
  });


  // Output results for view-based block-cyclic vector
  std::stringstream blk_cyc_vb_vec_ctr;
  blk_cyc_vb_vec_ctr << "Test : construct_vb_blk_cyc_vector\n"
              << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_vec_ctr << "Status : PASS\n";
  else
    blk_cyc_vb_vec_ctr << "Status : FAIL\n";
  report(blk_cyc_vb_vec_ctr, blk_cyc_vb_ctr_times);

  std::stringstream blk_cyc_vb_vec_gen;
  blk_cyc_vb_vec_gen << "Test : generate_on_vb_blk_cyc_vector\n"
              << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_vec_gen << "Status : PASS\n";
  else
    blk_cyc_vb_vec_gen << "Status : FAIL\n";
  report(blk_cyc_vb_vec_gen, blk_cyc_vb_gen_times);

  std::stringstream blk_cyc_vb_vec_acc;
  blk_cyc_vb_vec_acc << "Test : accumulate_on_vb_blk_cyc_vector\n"
              << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_vec_acc << "Status : PASS\n";
  else
    blk_cyc_vb_vec_acc << "Status : FAIL\n";
  report(blk_cyc_vb_vec_acc, blk_cyc_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << blk_cyc_vb_vec_ctr.str();
    std::cerr << blk_cyc_vb_vec_gen.str();
    std::cerr << blk_cyc_vb_vec_acc.str();
  });

  // Output results for view-based arbitrary vector
  std::stringstream arb_vb_vec_ctr;
  arb_vb_vec_ctr << "Test : construct_vb_arb_vector\n"
                 << "Version : STAPL\n";
  if (arb_vb_pass)
    arb_vb_vec_ctr << "Status : PASS\n";
  else
    arb_vb_vec_ctr << "Status : FAIL\n";
  report(arb_vb_vec_ctr, arb_vb_ctr_times);

  std::stringstream arb_vb_vec_gen;
  arb_vb_vec_gen << "Test : generate_on_vb_arb_vector\n"
                 << "Version : STAPL\n";
  if (arb_vb_pass)
    arb_vb_vec_gen << "Status : PASS\n";
  else
    arb_vb_vec_gen << "Status : FAIL\n";
  report(arb_vb_vec_gen, arb_vb_gen_times);

  std::stringstream arb_vb_vec_acc;
  arb_vb_vec_acc << "Test : accumulate_on_vb_arb_vector\n"
                 << "Version : STAPL\n";
  if (arb_vb_pass)
    arb_vb_vec_acc << "Status : PASS\n";
  else
    arb_vb_vec_acc << "Status : FAIL\n";
  report(arb_vb_vec_acc, arb_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << arb_vb_vec_ctr.str();
    std::cerr << arb_vb_vec_gen.str();
    std::cerr << arb_vb_vec_acc.str();
  });

  // Output results for composed view-based homogeneous-by-level vector
  std::stringstream comp_cyc_bal_blk_cyc_vb_ctr;
  comp_cyc_bal_blk_cyc_vb_ctr << "Test : construct_comp_homo_vb_vector\n"
                              << "Version : STAPL\n";
  if (comp_vb_pass)
    comp_vb_report << "Status : PASS\n";
  report(comp_cyc_bal_blk_cyc_vb_ctr, comp_vb_ctr_times);

  std::stringstream comp_vb_gen;
  comp_vb_gen << "Test : generate_on_comp_homo_vb_vector\n"
              << "Version : STAPL\n";
  if (comp_vb_pass)
    comp_vb_gen << "Status : PASS\n";
  report(comp_vb_gen, comp_vb_gen_times);

  std::stringstream comp_vb_acc;
  comp_vb_acc << "Test : accumulate_on_comp_homo_vb_vector\n"
              << "Version : STAPL\n";
  if (comp_vb_pass)
    comp_vb_acc << "Status : PASS\n";
  report(comp_vb_acc, comp_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << comp_cyc_bal_blk_cyc_vb_ctr.str();
    std::cerr << comp_vb_gen.str();
    std::cerr << comp_vb_acc.str();
  });

  return EXIT_SUCCESS;
}
