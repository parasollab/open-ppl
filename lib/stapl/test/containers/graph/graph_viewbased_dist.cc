/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <boost/lexical_cast.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/views/system_view.hpp>
#include "viewbased_dist_funcs.hpp"
#include "../../confint.hpp"

std::tuple<double, double, bool>
run_balanced(long int n, counter_t& timer, std::stringstream& o)
{
  timer.start();
  stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
    long int, long int> b(n);
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
    stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, long int, long int,
      stapl::view_based_partition<partitioning_view_type>,
      stapl::view_based_mapper<partitioning_view_type> > a(spec);
    timer.stop();
    return compute(a, o, default_value);
  }
  else
  {
    long int dv = 99;
    timer.start();
    stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, long int, long int,
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
  typedef stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, long int,
                               int, vb_part, vb_map> inner_cont_t;
  typedef stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                               inner_cont_t, int, vb_part, vb_map> mid_cont_t;
  typedef stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                               mid_cont_t, int, vb_part, vb_map> outer_cont_t;
  timer.start();
  outer_cont_t a(spec);
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

  // Controller for test of stapl::balanced partition
  counter_t bal_arr;
  confidence_interval_controller bal_gph_ctrl(samples, samples, 0.05);

  // Controller for tests of view-based distributions
  // Only one controller is used for simplicity.
  counter_t vb_arr;
  confidence_interval_controller bal_vb_gph_ctrl(samples, samples, 0.05);

  // Times for stapl::balanced partition
  std::vector<double> bal_gen_times;
  std::vector<double> bal_acc_times;
  std::stringstream bal_report;
  bal_report << "Test : graph_default_dist\n"
             << "Version : STAPL\n";
  bool bal_pass = true;

  // Times for balanced distribution
  std::vector<double> bal_vb_gen_times;
  std::vector<double> bal_vb_acc_times;
  std::stringstream bal_vb_report;
  bal_vb_report << "Test : graph_viewbased_balanced_dist\n"
                << "Version : STAPL\n";
  bool bal_vb_pass = true;

  // Times for block distribution
  std::vector<double> blk_vb_ctr_times;
  std::vector<double> blk_vb_gen_times;
  std::vector<double> blk_vb_acc_times;
  std::stringstream blk_vb_report;
  blk_vb_report << "Test : graph_viewbased_blocked_dist\n"
                << "Version : STAPL\n";
  bool blk_vb_pass = true;

  // Times for block-cyclic distribution
  std::vector<double> blk_cyc_vb_ctr_times;
  std::vector<double> blk_cyc_vb_gen_times;
  std::vector<double> blk_cyc_vb_acc_times;
  std::stringstream blk_cyc_vb_report;
  blk_cyc_vb_report << "Test : graph_viewbased_block_cyclic_dist\n"
                    << "Version : STAPL\n";
  bool blk_cyc_vb_pass = true;

  // Times for arbitrary distribution
  std::vector<double> arb_vb_ctr_times;
  std::vector<double> arb_vb_gen_times;
  std::vector<double> arb_vb_acc_times;
  std::stringstream arb_vb_report;
  arb_vb_report << "Test : graph_viewbased_arbitrary_dist\n"
                << "Version : STAPL\n";
  bool arb_vb_pass = true;

  // Times for composed cyclic/balanced/block-cyclic distribution
  std::vector<double> comp_vb_ctr_times;
  std::vector<double> comp_vb_gen_times;
  std::vector<double> comp_vb_acc_times;
  std::stringstream   comp_vb_report;
  comp_vb_report << "Test : homogeneous_comp_graph_dist\n"
                 << "Version : STAPL\n";
  bool comp_vb_pass = true;

  std::tuple<double, double, bool> alg_times;
  while (bal_gph_ctrl.iterate() || bal_vb_gph_ctrl.iterate())
  {
    // Test default distributed graph
    bal_arr.reset();
    alg_times = run_balanced(n, bal_arr, bal_report);
    bal_gph_ctrl.push_back(bal_arr.value());
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
    bal_vb_gph_ctrl.push_back(vb_arr.value());
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
    { stapl::cyclic(n1, stapl::current_level), stapl::balance(n2),
      stapl::block_cyclic(n3, comp_blk_sz, stapl::lowest_level) };
    vb_arr.reset();
    alg_times = run_vb_composed(dist_specs, vb_arr,
                                comp_vb_report);
    comp_vb_ctr_times.push_back(vb_arr.value());
    comp_vb_gen_times.push_back(std::get<0>(alg_times));
    comp_vb_acc_times.push_back(std::get<1>(alg_times));
    comp_vb_pass = comp_vb_pass && std::get<2>(alg_times);
  }


  // Output results for default balanced graph
  if (bal_pass)
    bal_report << "Status : PASS\n";
  else
    bal_report << "Status : FAIL\n";
  bal_gph_ctrl.report(bal_report);

  std::stringstream bal_gph_gen;
  bal_gph_gen << "Test : generate_on_bal_graph\n"
              << "Version : STAPL\n";
  if (bal_pass)
    bal_gph_gen << "Status : PASS\n";
  else
    bal_gph_gen << "Status : FAIL\n";
  report(bal_gph_gen, bal_gen_times);

  std::stringstream bal_gph_acc;
  bal_gph_acc << "Test : accumulate_on_bal_graph\n"
              << "Version : STAPL\n";
  if (bal_pass)
    bal_gph_acc << "Status : PASS\n";
  else
    bal_gph_acc << "Status : FAIL\n";
  report(bal_gph_acc, bal_acc_times);

  stapl::do_once([&](void) {
    std::cerr << bal_report.str();
    std::cerr << bal_gph_gen.str();
    std::cerr << bal_gph_acc.str();
  });


  // Output results for view-based balanced graph
  if (bal_vb_pass)
    bal_vb_report << "Status : PASS\n";
  else
    bal_vb_report << "Status : FAIL\n";
  bal_vb_gph_ctrl.report(bal_vb_report);

  std::stringstream bal_vb_gph_gen;
  bal_vb_gph_gen << "Test : generate_on_vb_bal_graph\n"
              << "Version : STAPL\n";
  if (bal_vb_pass)
    bal_vb_gph_gen << "Status : PASS\n";
  else
    bal_vb_gph_gen << "Status : FAIL\n";
  report(bal_vb_gph_gen, bal_vb_gen_times);

  std::stringstream bal_vb_gph_acc;
  bal_vb_gph_acc << "Test : accumulate_on_vb_bal_graph\n"
              << "Version : STAPL\n";
  if (bal_vb_pass)
    bal_vb_gph_acc << "Status : PASS\n";
  else
    bal_vb_gph_acc << "Status : FAIL\n";
  report(bal_vb_gph_acc, bal_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << bal_vb_report.str();
    std::cerr << bal_vb_gph_gen.str();
    std::cerr << bal_vb_gph_acc.str();
  });

  // Output results for view-based block graph
  std::stringstream blk_vb_gph_ctr;
  blk_vb_gph_ctr << "Test : construct_vb_blk_graph\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_gph_ctr << "Status : PASS\n";
  else
    blk_vb_gph_ctr << "Status : FAIL\n";
  report(blk_vb_gph_ctr, blk_vb_ctr_times);

  std::stringstream blk_vb_gph_gen;
  blk_vb_gph_gen << "Test : generate_on_vb_blk_graph\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_gph_gen << "Status : PASS\n";
  else
    blk_vb_gph_gen << "Status : FAIL\n";
  report(blk_vb_gph_gen, blk_vb_gen_times);

  std::stringstream blk_vb_gph_acc;
  blk_vb_gph_acc << "Test : accumulate_on_vb_blk_graph\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_gph_acc << "Status : PASS\n";
  else
    blk_vb_gph_acc << "Status : FAIL\n";
  report(blk_vb_gph_acc, blk_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << blk_vb_gph_ctr.str();
    std::cerr << blk_vb_gph_gen.str();
    std::cerr << blk_vb_gph_acc.str();
  });

  // Output results for view-based block-cyclic graph
  std::stringstream blk_cyc_vb_gph_ctr;
  blk_cyc_vb_gph_ctr << "Test : construct_vb_blk_cyc_graph\n"
              << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_gph_ctr << "Status : PASS\n";
  else
    blk_cyc_vb_gph_ctr << "Status : FAIL\n";
  report(blk_cyc_vb_gph_ctr, blk_cyc_vb_ctr_times);

  std::stringstream blk_cyc_vb_gph_gen;
  blk_cyc_vb_gph_gen << "Test : generate_on_vb_blk_cyc_graph\n"
              << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_gph_gen << "Status : PASS\n";
  else
    blk_cyc_vb_gph_gen << "Status : FAIL\n";
  report(blk_cyc_vb_gph_gen, blk_cyc_vb_gen_times);

  std::stringstream blk_cyc_vb_gph_acc;
  blk_cyc_vb_gph_acc << "Test : accumulate_on_vb_blk_cyc_graph\n"
              << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_gph_acc << "Status : PASS\n";
  else
    blk_cyc_vb_gph_acc << "Status : FAIL\n";
  report(blk_cyc_vb_gph_acc, blk_cyc_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << blk_cyc_vb_gph_ctr.str();
    std::cerr << blk_cyc_vb_gph_gen.str();
    std::cerr << blk_cyc_vb_gph_acc.str();
  });

  // Output results for view-based arbitrary graph
  std::stringstream arb_vb_gph_ctr;
  arb_vb_gph_ctr << "Test : construct_vb_arb_graph\n"
                 << "Version : STAPL\n";
  if (arb_vb_pass)
    arb_vb_gph_ctr << "Status : PASS\n";
  else
    arb_vb_gph_ctr << "Status : FAIL\n";
  report(arb_vb_gph_ctr, arb_vb_ctr_times);

  std::stringstream arb_vb_gph_gen;
  arb_vb_gph_gen << "Test : generate_on_vb_arb_graph\n"
                 << "Version : STAPL\n";
  if (arb_vb_pass)
    arb_vb_gph_gen << "Status : PASS\n";
  else
    arb_vb_gph_gen << "Status : FAIL\n";
  report(arb_vb_gph_gen, arb_vb_gen_times);

  std::stringstream arb_vb_gph_acc;
  arb_vb_gph_acc << "Test : accumulate_on_vb_arb_graph\n"
                 << "Version : STAPL\n";
  if (arb_vb_pass)
    arb_vb_gph_acc << "Status : PASS\n";
  else
    arb_vb_gph_acc << "Status : FAIL\n";
  report(arb_vb_gph_acc, arb_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << arb_vb_gph_ctr.str();
    std::cerr << arb_vb_gph_gen.str();
    std::cerr << arb_vb_gph_acc.str();
  });

  // Output results for composed view-based homogeneous-by-level graph
  std::stringstream comp_cyc_bal_blk_cyc_vb_ctr;
  comp_cyc_bal_blk_cyc_vb_ctr << "Test : construct_comp_homo_vb_graph\n"
                              << "Version : STAPL\n";
  if (comp_vb_pass)
    comp_cyc_bal_blk_cyc_vb_ctr << "Status : PASS\n";
  else
    comp_cyc_bal_blk_cyc_vb_ctr << "Status : FAIL\n";
  report(comp_cyc_bal_blk_cyc_vb_ctr, comp_vb_ctr_times);

  std::stringstream comp_vb_gen;
  comp_vb_gen << "Test : generate_on_comp_homo_vb_graph\n"
              << "Version : STAPL\n";
  if (comp_vb_pass)
    comp_vb_gen << "Status : PASS\n";
  else
    comp_vb_gen << "Status : FAIL\n";
  report(comp_vb_gen, comp_vb_gen_times);

  std::stringstream comp_vb_acc;
  comp_vb_acc << "Test : accumulate_on_comp_homo_vb_graph\n"
              << "Version : STAPL\n";
  if (comp_vb_pass)
    comp_vb_acc << "Status : PASS\n";
  else
    comp_vb_acc << "Status : FAIL\n";
  report(comp_vb_acc, comp_vb_acc_times);

  stapl::do_once([&](void) {
    std::cerr << comp_cyc_bal_blk_cyc_vb_ctr.str();
    std::cerr << comp_vb_gen.str();
    std::cerr << comp_vb_acc.str();
  });

  return EXIT_SUCCESS;
}
