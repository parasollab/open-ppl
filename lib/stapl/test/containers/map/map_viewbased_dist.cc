/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/algorithms/algorithm.hpp>

#include <stapl/map.hpp>
#include <stapl/vector.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/views/system_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/skeletons/utility/tags.hpp>

#include "../shared/viewbased_dist_funcs.hpp"
#include "../../confint.hpp"
#include "../../test_report.hpp"

using namespace stapl;

typedef stapl::counter<stapl::default_timer> counter_t;

struct populate_map
{
  typedef void result_type;

  template <typename Index, typename MapView>
  result_type operator()(Index i, MapView& m)
  { m[i] = i; }
};


struct get_second
{
  typedef long int result_type;

  template <typename Element>
  result_type operator()(Element e)
  { return e.second; }
};


template <typename Container>
std::tuple<double, double, bool>
compute(Container& c, long int n, std::stringstream& o)
{
  stapl::map_view<Container> cv(c);

  counter_t gen_timer, acc_timer;
  gen_timer.reset();
  acc_timer.reset();

  gen_timer.start();
  stapl::map_func(populate_map(), counting_view<size_t>(n),
                  make_repeat_view(cv));
  gen_timer.stop();

  // populate_map can result in RMI traffic as elements are inserted in the map.
  // ensure all messages have been processed so map is fully populated.
  rmi_fence();

  acc_timer.start();
  long int res = stapl::map_reduce(get_second(), plus<long int>(), cv);
  acc_timer.stop();

  // check that all locations received the same value.
  bool all_eq = all_locs_equal()(res);

  // check that the value received is correct.
  if (!all_eq || res != (n-1)*n/2)
  {
    o << "Status: FAIL\n";
    o << "Note: Error in computation. Expected " << (n-1)*n/2
      << " Computed " << res << " All loc equal " << all_eq << "\n";
    return std::make_tuple(gen_timer.value(), acc_timer.value(), false);
  }
  return std::make_tuple(gen_timer.value(), acc_timer.value(), true);
}


struct inner_mapwf
{
  typedef size_t result_type;

  // Element is a vector element
  template <typename U>
  result_type operator()(U const& x)
  { return x; }
};


struct outer_mapwf
{
  typedef size_t result_type;

  template <typename U>
  result_type operator()(U const& x)
  { return map_reduce(inner_mapwf(), plus<size_t>(), x.second); }
};


template <typename Container>
std::tuple<double, double, bool>
compute_composed(Container& m, long int n1, long int n2, std::stringstream& o)
{
  typedef map_view<Container> outer_view_type;

  location_type lid = get_location_id();
  location_type nlocs = get_num_locations();

  counter_t gen_timer, acc_timer;
  gen_timer.reset();
  acc_timer.reset();

  gen_timer.start();
  long int num_big_blocks = n1 % nlocs;
  long int block = n1 / nlocs + (lid < num_big_blocks ? 1 : 0);
  long int start = block * lid + (lid >= num_big_blocks ? num_big_blocks : 0);
  long int end   = start + block-1;

  for (long int i = start; i < end+1; ++i)
  {
    // m[i] here will create the inner vector using the composed distribution
    // specification provided.  Therefore the vector is properly sized, and a
    // call to resize isn't needed.
    for (long int j = 0; j != n2; ++j)
      m[i][j] = i*n2 + j;
  }
  // Wait on all locations to finish populating the container.
  rmi_fence();
  gen_timer.stop();
  bool corr_size = m.size() == (size_t)n1;

  acc_timer.start();
  outer_view_type v(m);
  long int res = map_reduce(outer_mapwf(), plus<long int>(), v);
  acc_timer.stop();

  // check that all locations received the same value.
  bool all_eq = all_locs_equal()(res);

  // ensure calls to size have finished before we return.
  rmi_fence();

  // check that the value received is correct.
  long int n = n1 * n2;
  if (!corr_size || !all_eq || res != (n-1)*n/2)
  {
    o << "Status: FAIL\n";
    o << "Note: Error in computation. Expected " << n*n*(n*n-1)/2
      << " Computed " << res << " All loc equal " << all_eq << "\n";
    return std::make_tuple(gen_timer.value(), acc_timer.value(), false);
  }
  return std::make_tuple(gen_timer.value(), acc_timer.value(), true);

}


std::tuple<double, double, bool>
run_balanced(long int n, counter_t& timer, std::stringstream& o)
{
  timer.start();
  indexed_domain<long int> dom(n);
  map<long int, long int> b(dom);
  timer.stop();
  return compute(b, n, o);
}


std::tuple<double, double, bool>
run_vb(distribution_spec<>& spec, long int n, counter_t& timer,
       std::stringstream& o)
{
  typedef distribution_spec<> partitioning_view_type;
  timer.start();
  map<long int, long int, less<long int>,
    view_based_partition<partitioning_view_type>,
    view_based_mapper<partitioning_view_type> > a(spec);
  timer.stop();
  return compute(a, n, o);
}


template <typename PartInfoContainer>
std::tuple<double, double, bool>
run_cb(PartInfoContainer& part_info, long int n, counter_t& timer,
       std::stringstream& o)
{
  typedef distribution_spec<> partitioning_view_type;
  timer.start();
  map<long int, long int, less<long int>,
    stapl::view_based_partition<partitioning_view_type, PartInfoContainer>,
    stapl::view_based_mapper<partitioning_view_type> > a(part_info);
  timer.stop();
  return compute(a, n, o);
}


template <typename DistSpec>
std::tuple<double, double, bool>
run_vb_composed(std::vector<DistSpec> const& dist_specs, long int n1,
                long int n2, counter_t& timer, std::stringstream& o)
{
  typedef view_based_partition<distribution_spec<>> vb_partition;
  typedef view_based_mapper<distribution_spec<>>    vb_mapping;

  // Test composed distribution constructors using map<size_t, vector<size_t> >
  typedef vector<size_t, vb_partition, vb_mapping>  inner_container_type;

  typedef map<size_t, inner_container_type, less<size_t>,
              vb_partition, vb_mapping>     composed_container_type;
  timer.start();
  composed_container_type cm1(dist_specs);
  timer.stop();
  return compute_composed(cm1, n1, n2, o);
}

exit_code stapl_main(int argc, char* argv[])
{
  long int n, n1, n2;
  long int blk_sz, comp_blk_sz;
  int samples;
  if (argc > 6)
  {
    n = boost::lexical_cast<long int>(argv[1]);
    n1 = boost::lexical_cast<long int>(argv[2]);
    n2 = boost::lexical_cast<long int>(argv[3]);
    blk_sz = boost::lexical_cast<long int>(argv[4]);
    comp_blk_sz = boost::lexical_cast<long int>(argv[5]);
    samples = boost::lexical_cast<long int>(argv[6]);
  }
  else
  {
    n = 1024;
    n1 = 32;
    n2 = 32;
    blk_sz = n / 64;
    comp_blk_sz = n2 / 8;
    samples = 1;
  }

  if (blk_sz == 0)
  {
    printf("n too small.  Must be at least %d\n",512*16);
    return EXIT_FAILURE;
  }

  // Controller for test of stapl::balanced partition
  counter_t bal_arr;
  confidence_interval_controller bal_map_ctrl(samples, samples, 0.05);

  // Controller for tests of view-based distributions
  // Only one controller is used for simplicity.
  counter_t vb_arr;
  confidence_interval_controller bal_vb_map_ctrl(samples, samples, 0.05);

  // Times for stapl::balanced partition
  std::vector<double> bal_gen_times;
  std::vector<double> bal_acc_times;
  std::stringstream bal_report;
  bal_report << "Test : map_default_dist\n"
             << "Version : STAPL\n";
  bool bal_pass = true;

  // Times for balanced distribution
  std::vector<double> bal_vb_gen_times;
  std::vector<double> bal_vb_acc_times;
  std::stringstream bal_vb_report;
  bal_vb_report << "Test : map_viewbased_balanced_dist\n"
                << "Version : STAPL\n";
  bool bal_vb_pass = true;

  // Times for block distribution
  std::vector<double> blk_vb_ctr_times;
  std::vector<double> blk_vb_gen_times;
  std::vector<double> blk_vb_acc_times;
  std::stringstream blk_vb_report;
  blk_vb_report << "Test : map_viewbased_blocked_dist\n"
                << "Version : STAPL\n";
  bool blk_vb_pass = true;

  // Times for block-cyclic distribution
  std::vector<double> blk_cyc_vb_ctr_times;
  std::vector<double> blk_cyc_vb_gen_times;
  std::vector<double> blk_cyc_vb_acc_times;
  std::stringstream blk_cyc_vb_report;
  blk_cyc_vb_report << "Test : map_viewbased_block_cyclic_dist\n"
                    << "Version : STAPL\n";
  bool blk_cyc_vb_pass = true;

  // Times for arbitrary distribution
  std::vector<double> arb_vb_ctr_times;
  std::vector<double> arb_vb_gen_times;
  std::vector<double> arb_vb_acc_times;
  std::stringstream arb_vb_report;
  arb_vb_report << "Test : map_viewbased_arbitrary_dist\n"
                << "Version : STAPL\n";
  bool arb_vb_pass = true;

  // Times for composed cyclic/balanced/block-cyclic distribution
  std::vector<double> comp_vb_ctr_times;
  std::vector<double> comp_vb_gen_times;
  std::vector<double> comp_vb_acc_times;
  std::stringstream   comp_vb_report;
  comp_vb_report << "Test : homogeneous_comp_map_vec_dist\n"
                 << "Version : STAPL\n";
  bool comp_vb_pass = true;

  // Times for container-based arbitrary distribution
  std::vector<double> cb_arb_vb_ctr_times;
  std::vector<double> cb_arb_vb_gen_times;
  std::vector<double> cb_arb_vb_acc_times;
  std::stringstream cb_arb_vb_report;
  cb_arb_vb_report << "Test : map_viewbased_cb_arbitrary_dist\n"
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
  while (bal_map_ctrl.iterate() || bal_vb_map_ctrl.iterate())
  {
    // Test default distributed map
    bal_arr.reset();
    alg_times = run_balanced(n, bal_arr, bal_report);
    bal_map_ctrl.push_back(bal_arr.value());
    bal_gen_times.push_back(std::get<0>(alg_times));
    bal_acc_times.push_back(std::get<1>(alg_times));
    bal_pass = bal_pass && std::get<2>(alg_times);

    // Test view-based balanced distribution
    distribution_spec<> bal_spec = balance(n);

    vb_arr.reset();
    // construction uses a default value, which requires traversing components.
    // This should result in slightly slower construction, but still faster than
    // the other constructors.
    alg_times = run_vb(bal_spec, n, vb_arr, bal_vb_report);
    bal_vb_map_ctrl.push_back(vb_arr.value());
    bal_vb_gen_times.push_back(std::get<0>(alg_times));
    bal_vb_acc_times.push_back(std::get<1>(alg_times));
    bal_vb_pass = bal_vb_pass && std::get<2>(alg_times);

    // Test view-based blocked distribution
    distribution_spec<> blk_spec = block(n, blk_sz);

    vb_arr.reset();
    alg_times = run_vb(blk_spec, n, vb_arr, blk_vb_report);
    blk_vb_ctr_times.push_back(vb_arr.value());
    blk_vb_gen_times.push_back(std::get<0>(alg_times));
    blk_vb_acc_times.push_back(std::get<1>(alg_times));
    blk_vb_pass = blk_vb_pass && std::get<2>(alg_times);

    // Test view-based block-cyclic distribution
    distribution_spec<> blk_cyc_spec =
      block_cyclic(n, blk_sz);

    vb_arr.reset();
    alg_times = run_vb(blk_cyc_spec, n, vb_arr, blk_cyc_vb_report);
    blk_cyc_vb_ctr_times.push_back(vb_arr.value());
    blk_cyc_vb_gen_times.push_back(std::get<0>(alg_times));
    blk_cyc_vb_acc_times.push_back(std::get<1>(alg_times));
    blk_cyc_vb_pass = blk_cyc_vb_pass && std::get<2>(alg_times);

    // Test view-based arbitrary distribution
    uneven_blocks ueb;
    reverse_parts rp(get_num_locations(), get_num_locations());
    distribution_spec<> arb_spec = arbitrary(n, get_num_locations(),
                                                  ueb, rp);
    vb_arr.reset();
    alg_times = run_vb(arb_spec, n, vb_arr, arb_vb_report);
    arb_vb_ctr_times.push_back(vb_arr.value());
    arb_vb_gen_times.push_back(std::get<0>(alg_times));
    arb_vb_acc_times.push_back(std::get<1>(alg_times));
    arb_vb_pass = arb_vb_pass && std::get<2>(alg_times);

    // Test homogeneous by level view-based distribution
    std::vector<distribution_spec<>> dist_specs =
      { cyclic(n1), block_cyclic(n2, comp_blk_sz, current_level) };
    vb_arr.reset();
    alg_times = run_vb_composed(dist_specs, n1, n2, vb_arr, comp_vb_report);
    comp_vb_ctr_times.push_back(vb_arr.value());
    comp_vb_gen_times.push_back(std::get<0>(alg_times));
    comp_vb_acc_times.push_back(std::get<1>(alg_times));
    comp_vb_pass = comp_vb_pass && std::get<2>(alg_times);

    // Test view-based arbitrary distribution that uses a container of partition
    // information.
    vb_arr.reset();
    alg_times = run_cb(part_view, n, vb_arr, cb_arb_vb_report);
    cb_arb_vb_ctr_times.push_back(vb_arr.value());
    cb_arb_vb_gen_times.push_back(std::get<0>(alg_times));
    cb_arb_vb_acc_times.push_back(std::get<1>(alg_times));
    cb_arb_vb_pass = cb_arb_vb_pass && std::get<2>(alg_times);
  }


  // Output results for default balanced map
  if (bal_pass)
    bal_report << "Status : PASS\n";
  else
    bal_report << "Status : FAIL\n";
  bal_map_ctrl.report(bal_report);

  std::stringstream bal_map_gen;
  bal_map_gen << "Test : generate_on_bal_map\n"
              << "Version : STAPL\n";
  if (bal_pass)
    bal_map_gen << "Status : PASS\n";
  else
    bal_map_gen << "Status : FAIL\n";
  report(bal_map_gen, bal_gen_times);

  std::stringstream bal_map_acc;
  bal_map_acc << "Test : accumulate_on_bal_map\n"
              << "Version : STAPL\n";
  if (bal_pass)
    bal_map_acc << "Status : PASS\n";
  else
    bal_map_acc << "Status : FAIL\n";
  report(bal_map_acc, bal_acc_times);

  do_once([&](void) {
    std::cerr << bal_report.str();
    std::cerr << bal_map_gen.str();
    std::cerr << bal_map_acc.str();
  });


  // Output results for view-based balanced map
  if (bal_vb_pass)
    bal_vb_report << "Status : PASS\n";
  else
    bal_vb_report << "Status : FAIL\n";
  bal_vb_map_ctrl.report(bal_vb_report);

  std::stringstream bal_vb_map_gen;
  bal_vb_map_gen << "Test : generate_on_vb_bal_map\n"
              << "Version : STAPL\n";
  if (bal_vb_pass)
    bal_vb_map_gen << "Status : PASS\n";
  else
    bal_vb_map_gen << "Status : FAIL\n";
  report(bal_vb_map_gen, bal_vb_gen_times);

  std::stringstream bal_vb_map_acc;
  bal_vb_map_acc << "Test : accumulate_on_vb_bal_map\n"
              << "Version : STAPL\n";
  if (bal_vb_pass)
    bal_vb_map_acc << "Status : PASS\n";
  else
    bal_vb_map_acc << "Status : FAIL\n";
  report(bal_vb_map_acc, bal_vb_acc_times);

  do_once([&](void) {
    std::cerr << bal_vb_report.str();
    std::cerr << bal_vb_map_gen.str();
    std::cerr << bal_vb_map_acc.str();
  });


  // Output results for view-based block map
  std::stringstream blk_vb_map_ctr;
  blk_vb_map_ctr << "Test : construct_vb_blk_map\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_map_ctr << "Status : PASS\n";
  else
    blk_vb_map_ctr << "Status : FAIL\n";
  report(blk_vb_map_ctr, blk_vb_ctr_times);

  std::stringstream blk_vb_map_gen;
  blk_vb_map_gen << "Test : generate_on_vb_blk_map\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_map_gen << "Status : PASS\n";
  else
    blk_vb_map_gen << "Status : FAIL\n";
  report(blk_vb_map_gen, blk_vb_gen_times);

  std::stringstream blk_vb_map_acc;
  blk_vb_map_acc << "Test : accumulate_on_vb_blk_map\n"
              << "Version : STAPL\n";
  if (blk_vb_pass)
    blk_vb_map_acc << "Status : PASS\n";
  else
    blk_vb_map_acc << "Status : FAIL\n";
  report(blk_vb_map_acc, blk_vb_acc_times);

  do_once([&](void) {
    std::cerr << blk_vb_map_ctr.str();
    std::cerr << blk_vb_map_gen.str();
    std::cerr << blk_vb_map_acc.str();
  });


  // Output results for view-based block-cyclic map
  std::stringstream blk_cyc_vb_map_ctr;
  blk_cyc_vb_map_ctr << "Test : construct_vb_blk_cyc_map\n"
              << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_map_ctr << "Status : PASS\n";
  else
    blk_cyc_vb_map_ctr << "Status : FAIL\n";
  report(blk_cyc_vb_map_ctr, blk_cyc_vb_ctr_times);

  std::stringstream blk_cyc_vb_map_gen;
  blk_cyc_vb_map_gen << "Test : generate_on_vb_blk_cyc_map\n"
              << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_map_gen << "Status : PASS\n";
  else
    blk_cyc_vb_map_gen << "Status : FAIL\n";
  report(blk_cyc_vb_map_gen, blk_cyc_vb_gen_times);

  std::stringstream blk_cyc_vb_map_acc;
  blk_cyc_vb_map_acc << "Test : accumulate_on_vb_blk_cyc_map\n"
              << "Version : STAPL\n";
  if (blk_cyc_vb_pass)
    blk_cyc_vb_map_acc << "Status : PASS\n";
  else
    blk_cyc_vb_map_acc << "Status : FAIL\n";
  report(blk_cyc_vb_map_acc, blk_cyc_vb_acc_times);

  do_once([&](void) {
    std::cerr << blk_cyc_vb_map_ctr.str();
    std::cerr << blk_cyc_vb_map_gen.str();
    std::cerr << blk_cyc_vb_map_acc.str();
  });

  // Output results for view-based arbitrary map
  std::stringstream arb_vb_map_ctr;
  arb_vb_map_ctr << "Test : construct_vb_arb_map\n"
                 << "Version : STAPL\n";
  if (arb_vb_pass)
    arb_vb_map_ctr << "Status : PASS\n";
  else
    arb_vb_map_ctr << "Status : FAIL\n";
  report(arb_vb_map_ctr, arb_vb_ctr_times);

  std::stringstream arb_vb_map_gen;
  arb_vb_map_gen << "Test : generate_on_vb_arb_map\n"
                 << "Version : STAPL\n";
  if (arb_vb_pass)
    arb_vb_map_gen << "Status : PASS\n";
  else
    arb_vb_map_gen << "Status : FAIL\n";
  report(arb_vb_map_gen, arb_vb_gen_times);

  std::stringstream arb_vb_map_acc;
  arb_vb_map_acc << "Test : accumulate_on_vb_arb_map\n"
                 << "Version : STAPL\n";
  if (arb_vb_pass)
    arb_vb_map_acc << "Status : PASS\n";
  else
    arb_vb_map_acc << "Status : FAIL\n";
  report(arb_vb_map_acc, arb_vb_acc_times);

  do_once([&](void) {
    std::cerr << arb_vb_map_ctr.str();
    std::cerr << arb_vb_map_gen.str();
    std::cerr << arb_vb_map_acc.str();
  });

  // Output results for composed view-based homogeneous-by-level map
  std::stringstream comp_cyc_bal_blk_cyc_vb_ctr;
  comp_cyc_bal_blk_cyc_vb_ctr << "Test : construct_comp_hetero_vb_map\n"
                              << "Version : STAPL\n";
  if (comp_vb_pass)
    comp_vb_report << "Status : PASS\n";
  report(comp_cyc_bal_blk_cyc_vb_ctr, comp_vb_ctr_times);

  std::stringstream comp_vb_gen;
  comp_vb_gen << "Test : generate_on_comp_hetero_vb_map\n"
              << "Version : STAPL\n";
  if (comp_vb_pass)
    comp_vb_gen << "Status : PASS\n";
  report(comp_vb_gen, comp_vb_gen_times);

  std::stringstream comp_vb_acc;
  comp_vb_acc << "Test : accumulate_on_comp_hetero_vb_map\n"
              << "Version : STAPL\n";
  if (comp_vb_pass)
    comp_vb_acc << "Status : PASS\n";
  report(comp_vb_acc, comp_vb_acc_times);

  do_once([&](void) {
    std::cerr << comp_cyc_bal_blk_cyc_vb_ctr.str();
    std::cerr << comp_vb_gen.str();
    std::cerr << comp_vb_acc.str();
  });

  // Output results for view-based container-based arbitrary map
  std::stringstream cb_arb_vb_arr_ctr;
  cb_arb_vb_arr_ctr << "Test : construct_vb_cb_arb_map\n"
                    << "Version : STAPL\n";
  if (cb_arb_vb_pass)
    cb_arb_vb_arr_ctr << "Status : PASS\n";
  else
    cb_arb_vb_arr_ctr << "Status : FAIL\n";
  report(cb_arb_vb_arr_ctr, cb_arb_vb_ctr_times);

  std::stringstream cb_arb_vb_arr_gen;
  cb_arb_vb_arr_gen << "Test : generate_on_vb_cb_arb_map\n"
                    << "Version : STAPL\n";
  if (cb_arb_vb_pass)
    cb_arb_vb_arr_gen << "Status : PASS\n";
  else
    cb_arb_vb_arr_gen << "Status : FAIL\n";
  report(cb_arb_vb_arr_gen, cb_arb_vb_gen_times);

  std::stringstream cb_arb_vb_arr_acc;
  cb_arb_vb_arr_acc << "Test : accumulate_on_cb_vb_arb_map\n"
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
