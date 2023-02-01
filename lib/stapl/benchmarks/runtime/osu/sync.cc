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
/// Benchmark for @ref stapl::rmi_barrier() / @ref stapl::rmi_fence() latency.
///
/// The main idea of the benchmark is to perform the synchronization operation
/// a few times and measure average, min and max times across all locations.
///
/// It can be compared against
/// -# @c mpi/collective/osu_barrier.c
/// -# @c openshmem/osu_oshm_barrier.c
/// in OSU benchmarks.
//////////////////////////////////////////////////////////////////////

#define BENCHMARK "# %s Latency Benchmark\n"

#include <stapl/runtime.hpp>
#include <cstdio>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include "common.hpp"

#define SKIP       10
#define ITERATIONS 100

#ifndef FIELD_WIDTH
# define FIELD_WIDTH 20
#endif

#ifndef FLOAT_PRECISION
# define FLOAT_PRECISION 2
#endif


using namespace stapl;


exit_code stapl_main(int argc, char *argv[])
{
  if (get_num_locations() < 2) {
    if (get_location_id() == 0)
      error("%s: This benchmark requires at least two locations.\n", argv[0]);
    return EXIT_FAILURE;
  }

  int skip = SKIP;

  // options
  int iterations   = ITERATIONS;
  bool show_full   = false;
  rmi_primitive pr = rmi_primitive::UNKNOWN;

  // parse options
  try {
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
      ("help",
       "Print this help message")
      ("i",
       po::value<int>(),
       "Iterations per payload size (default is 100")
      ("f",
       "Print mix/max latency and iterations")
      ("primitive",
       po::value<std::string>()->default_value("rmi_fence"),
       "Primitive to be used (rmi_barrier, rmi_fence)");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      if (get_location_id()==0)
        std::cout << desc << std::endl;
      return EXIT_SUCCESS;
    }

    if (vm.count("i")) {
      iterations = vm["i"].as<int>();
    }

    if (vm.count("f"))
      show_full = true;

    if (vm.count("primitive")) {
      std::string const& s = vm["primitive"].as<std::string>();
      if (s=="rmi_barrier") {
        pr = rmi_primitive::BARRIER;
      }
      else if (s=="rmi_fence") {
        pr = rmi_primitive::FENCE;
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

  if (get_location_id() == 0) {
    std::fprintf(stdout, BENCHMARK, get_primitive_name(pr));
    std::fprintf(stdout, "# %s", "Avg Latency (us)");
    if (show_full) {
      std::fprintf(stdout, "%*s", FIELD_WIDTH, "Min Latency(us)");
      std::fprintf(stdout, "%*s", FIELD_WIDTH, "Max Latency(us)");
      std::fprintf(stdout, "%*s\n", 12, "Iterations");
    }
    else {
      std::fprintf(stdout, "\n");
    }
    std::fflush(stdout);
  }

  distributed_value<double> latencies(true);

  counter<default_timer> counter;
  double time = 0.0;

  rmi_fence(); // quiescence before benchmarking

  switch (pr) {
    case rmi_primitive::BARRIER:
      for (int i = 0; i < skip + iterations; i++) {
        counter.reset();
        counter.start();
        rmi_barrier();
        const double t = counter.stop();
        if (i>=skip) {
          time += t;
        }
        rmi_fence(); // wait for collective on all locations
      }
      break;
    case rmi_primitive::FENCE:
      for (int i = 0; i < skip + iterations; i++) {
        counter.reset();
        counter.start();
        rmi_fence();
        const double t = counter.stop();
        if (i>=skip) {
          time += t;
        }
        rmi_fence(); // wait for collective on all locations
      }
      break;
    default:
      error("%s: Incorrect primitive.\n", argv[0]);
      break;
  }

  const double latency = ((time * 1e6) / double(iterations));
  latencies.set_value(latency);
  rmi_fence(); // wait for set_value to finish on all locations

  if (get_location_id()==0) {
    const double avg_time = (latencies.reduce(std::plus<double>()) /
                             get_num_locations());
    std::fprintf(stdout, "%*.*f", 17, FLOAT_PRECISION, avg_time);
    if (show_full) {
      const double min_time = latencies.reduce(
                                [](double const& x, double const& y)
                                { return double(std::min(x, y)); });
      const double max_time = latencies.reduce(
                                [](double const& x, double const& y)
                                { return double(std::max(x, y)); });
      std::fprintf(stdout, "%*.*f%*.*f%*d\n",
                   FIELD_WIDTH, FLOAT_PRECISION, min_time,
                   FIELD_WIDTH, FLOAT_PRECISION, max_time,
                   12, iterations);
    }
    else {
      std::fprintf(stdout, "\n");
    }
    std::fflush(stdout);
  }

  rmi_fence(); // wait before destroying the p_objects

  return EXIT_SUCCESS;
}
