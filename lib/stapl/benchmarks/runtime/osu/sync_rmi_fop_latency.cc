/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Benchmark for @ref stapl::sync_rmi() / @ref stapl::opaque_rmi() /
/// @ref stapl::promise fetch-and-add latency.
///
/// The main idea of the benchmark is to do a fetch-and-add on the remote side
/// and return the result of the operation.
///
/// It can be compared against
/// -# @c mpi/one-sided/osu_fop_latency.c
/// in OSU benchmarks.
///
/// @note This benchmark is currently only defined for a <tt>long long</tt>.
//////////////////////////////////////////////////////////////////////

#define BENCHMARK "# %s fetch-and-op Benchmark\n"

#include <stapl/runtime.hpp>
#include <cstdio>
#include <iostream>
#include <string>
#include <unistd.h>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/utility/in_place_factory.hpp>
#include "common.hpp"

#ifndef FIELD_WIDTH
# define FIELD_WIDTH 20
#endif

#ifndef FLOAT_PRECISION
# define FLOAT_PRECISION 2
#endif


using namespace stapl;


exit_code stapl_main(int argc, char *argv[])
{
  if (get_num_locations() != 2) {
    if (get_location_id() == 0)
      error("%s: This benchmark requires two locations.\n", argv[0]);
    return EXIT_FAILURE;
  }

  int skip = 10;
  int loop = 500;

  // options
  bool aggregation   = false;
  bool src_diff_gang = false;
  rmi_primitive pr   = rmi_primitive::UNKNOWN;

  // parse options
  try {
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
      ("help",
       "Print this help message")
      ("aggregation",
       "Enable aggregation")
      ("src_diff_gang",
       "RMI source location is in a different gang")
      ("primitive",
       po::value<std::string>()->default_value("sync_rmi"),
       "Primitive to be used (sync_rmi, opaque_rmi, promise)");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      if (get_location_id()==0)
        std::cout << desc << std::endl;
      return EXIT_SUCCESS;
    }

    if (vm.count("aggregation"))
      aggregation = true;

    if (vm.count("src_diff_gang"))
      src_diff_gang = true;

    if (vm.count("primitive")) {
      std::string const& s = vm["primitive"].as<std::string>();
      if (s=="sync_rmi") {
        pr = rmi_primitive::SYNC_RMI;
      }
      else if (s=="opaque_rmi") {
        pr = rmi_primitive::OPAQUE_RMI;
      }
      else if (s=="promise") {
        pr = rmi_primitive::PROMISE_WITH_ASYNC_RMI;
      }
      else {
        if (get_location_id() == 0)
          error("%s: Incorrect primitive.\n", argv[0]);
        return EXIT_FAILURE;
      }
    }
  }
  catch (boost::program_options::unknown_option& ex) {
    error("%s: %s.\n", argv[0], ex.what());
  }

  typedef long long                     value_type;
  typedef distributed_value<value_type> dist_T;

  // create p_object
  dist_T d_obj(aggregation);

  if (get_location_id() == 0) {
    std::fprintf(stdout, BENCHMARK, get_primitive_name(pr));
    std::fprintf(stdout, "# Aggregation: %s\n", (aggregation ? "yes" : "no"));
    std::fprintf(stdout, "# Intragang: %s\n", (src_diff_gang ? "no" : "yes"));
    std::fprintf(stdout, "%-*s%*s\n",
                 10, "# Size", FIELD_WIDTH, "Latency (us)");
    std::fflush(stdout);
  }

  // latency benchmark
  rmi_handle::reference const& r = d_obj.get_rmi_handle();
  counter<default_timer> counter;

  rmi_fence(); // quiescence before benchmarking

  if (get_location_id() == 0) {
    // if requested, generate traffic from its own gang
    boost::optional<gang> g;
    if (src_diff_gang)
      g = boost::in_place();

    switch (pr) {
      case rmi_primitive::SYNC_RMI:
        for (int i = 0; i < skip + loop; i++) {
          if (i == skip)
            counter.start();

          sync_rmi(1, r, &dist_T::fetch_and_add, 0);
        }
        break;
      case rmi_primitive::OPAQUE_RMI:
        for (int i = 0; i < skip + loop; i++) {
          if (i == skip)
            counter.start();

          future<value_type> f = opaque_rmi(1, r, &dist_T::fetch_and_add, 0);
          f.get();
        }
        break;
      case rmi_primitive::PROMISE_WITH_ASYNC_RMI:
        for (int i = 0; i < skip + loop; i++) {
          if (i == skip)
            counter.start();

          promise<value_type> p;
          auto f = p.get_future();
          async_rmi(1, r, &dist_T::fetch_and_add_promise, std::move(p), 0);
          f.get();
        }
        break;
      default:
        error("%s: Incorrect primitive.\n", argv[0]);
        break;
    }
    const double t = counter.stop();
    const double latency = (t * 1.0e6 / loop);

    const int size = sizeof(value_type);
    std::fprintf(stdout, "%-*d%*.*f\n",
                 10, size, FIELD_WIDTH, FLOAT_PRECISION, latency);
    std::fflush(stdout);

    // finish all traffic from newly created gang
    if (g)
      rmi_fence();
  }

  rmi_fence(); // wait for RMI calls to finish

  return EXIT_SUCCESS;
}
