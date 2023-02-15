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
/// Benchmark for @ref stapl::allgather_rmi() / @ref stapl::allreduce_rmi() /
/// @ref stapl::broadcast_rmi() latency.
///
/// The main idea of the benchmark is to perform the SPMD collective operation
/// for a few iterations and measure average, min and max times across all
/// locations.
///
/// It can be compared against
/// -# @c mpi/collective/osu_allgather.c ("--primitive allgather_rmi" flag)
/// -# @c mpi/collective/osu_allgatherv.c ("--primitive allgather_rmi" flag)
/// -# @c mpi/collective/osu_allreduce.c ("--primitive allreduce_rmi" flag)
/// -# @c mpi/collective/osu_bcast.c ("--primitive broadcast_rmi" flag)
/// -# @c openshmem/osu_oshm_broadcast.c ("--primitive broadcast_rmi" flag)
/// -# @c openshmem/osu_oshm_reduce.c ("--primitive allreduce_rmi" flag)
/// in OSU benchmarks.
//////////////////////////////////////////////////////////////////////

#define BENCHMARK "# %s Latency Benchmark\n"

#include <stapl/runtime.hpp>
#include <cstdio>
#include <functional>
#include <iostream>
#include <string>
#include <valarray>
#include <unistd.h>
#include <boost/program_options.hpp>
#include "common.hpp"

#ifndef DEFAULT_MAX_SIZE
# define DEFAULT_MAX_SIZE (1 << 20)
#endif

#define SKIP             200
#define SKIP_LARGE       10
#define LARGE_SIZE       8192
#define MAX_ALIGNMENT    16384
#define ITERATIONS       1000
#define ITERATIONS_LARGE 100

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
  int max_size         = DEFAULT_MAX_SIZE;
  int iterations       = ITERATIONS;
  int iterations_large = ITERATIONS_LARGE;
  bool show_full       = false;
  bool aggregation     = false;
  rmi_primitive pr     = rmi_primitive::UNKNOWN;

  // parse options
  try {
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
      ("help",
       "Print this help message")
      ("m",
       po::value<int>(),
       "Maximum message size in bytes (default is 1MB)")
      ("i",
       po::value<int>(),
       "Iterations per payload size (default is 1000 for small payloads, 100 "
       "for large payloads")
      ("f",
       "Print mix/max latency and iterations")
      ("aggregation",
       "Enable aggregation")
      ("primitive",
       po::value<std::string>()->default_value("allreduce_rmi"),
       "Primitive to be used (allgather_rmi, allreduce_rmi, broadcast_rmi)");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      if (get_location_id()==0)
        std::cout << desc << std::endl;
      return EXIT_SUCCESS;
    }

    if (vm.count("m"))
      max_size = vm["m"].as<int>();

    if (vm.count("i")) {
      iterations       = vm["i"].as<int>();
      iterations_large = iterations;
    }

    if (vm.count("f"))
      show_full = true;

    if (vm.count("aggregation"))
      aggregation = true;

    if (vm.count("primitive")) {
      std::string const& s = vm["primitive"].as<std::string>();
      if (s=="allgather_rmi") {
        pr = rmi_primitive::ALLGATHER_RMI;
      }
      else if (s=="allreduce_rmi") {
        pr = rmi_primitive::ALLREDUCE_RMI;
      }
      else if (s=="broadcast_rmi") {
        pr = rmi_primitive::BROADCAST_RMI;
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

  // allocate payload buffer
  const int page_size = getpagesize();
  assert(page_size <= MAX_ALIGNMENT);
  char* buf   = new char[max_size + MAX_ALIGNMENT];
  char* s_buf = static_cast<char*>(align_buffer(buf, page_size));

  // create p_object for data movement benchmarks
  unsigned int flags = 0;
  if (!aggregation)
    flags |= no_aggregation;

  distributed_object d_obj(flags, s_buf);

  // create p_object for reduction benchmarks
  typedef std::valarray<float> array_type;
  distributed_value<array_type> d_val(aggregation);

  if (get_location_id() == 0) {
    std::fprintf(stdout, BENCHMARK, get_primitive_name(pr));
    std::fprintf(stdout, "# Aggregation: %s\n", (aggregation ? "yes" : "no"));
    std::fprintf(stdout, "%-*s%*s",
                 10, "# Size", FIELD_WIDTH, "Avg Latency (us)");
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

  rmi_handle::reference const& r  = d_obj.get_rmi_handle();
  rmi_handle::reference const& rv = d_val.get_rmi_handle();

  for (int size = 1; size <= max_size; size *= 2) {
    if (size > LARGE_SIZE) {
      skip       = SKIP_LARGE;
      iterations = iterations_large;
    }

    counter<default_timer> counter;
    double time = 0.0;

    rmi_fence(); // quiescence before benchmarking

    switch (pr) {
      case rmi_primitive::ALLGATHER_RMI:
        for (int i = 0; i < skip + iterations; i++) {
          counter.reset();
          counter.start();
          allgather_rmi(r, &distributed_object::get_nodisp, size).get();
          const double t = counter.stop();
          if (i>=skip) {
            time += t;
          }
          rmi_fence(); // wait for RMI calls to finish
        }
        break;
      case rmi_primitive::ALLREDUCE_RMI: {
        if (int(size * sizeof(float)) > max_size) {
          // exit from loop
          size = max_size;
          continue;
        }

        d_val.set_value(array_type(size));
        rmi_fence(); // wait for set_value to finish on all locations

        for (int i = 0; i < skip + iterations; i++) {
          counter.reset();
          counter.start();
          allreduce_rmi(std::plus<array_type>(),
                        rv, &distributed_value<array_type>::get_value).get();
          const double t = counter.stop();
          if (i>=skip) {
            time += t;
          }
          rmi_fence(); // wait for RMI calls to finish
        }
      } break;
      case rmi_primitive::BROADCAST_RMI: {
        const bool root = (get_location_id() == 0);
        for (int i = 0; i < skip + iterations; i++) {
          counter.reset();
          counter.start();
          if (root)
            broadcast_rmi(root_location, r,
                          &distributed_object::get_nodisp, size).get();
          else
            broadcast_rmi(0, &distributed_object::get_nodisp).get();
          const double t = counter.stop();
          if (i>=skip) {
            time += t;
          }
          rmi_fence(); // wait for RMI calls to finish
        }
      } break;
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
      std::fprintf(stdout, "%-*d", 10, size);
      std::fprintf(stdout, "%*.*f", FIELD_WIDTH, FLOAT_PRECISION, avg_time);
      if (show_full) {
        const double min_time = latencies.reduce(
                                  [](double const& x, double const& y)
                                  { return double(std::min(x,y)); });
        const double max_time = latencies.reduce(
                                  [](double const& x, double const& y)
                                  { return double(std::max(x,y)); });
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
  }

  rmi_fence(); // wait for RMI calls to finish

  delete[] buf;

  return EXIT_SUCCESS;
}
