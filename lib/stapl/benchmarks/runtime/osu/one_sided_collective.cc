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
/// Benchmark for @ref stapl::async_rmi() / @ref stapl::opaque_rmi() /
/// @ref stapl::reduce_rmi() (and their unordered variants) to all locations
/// latency.
///
/// The main idea of the benchmark is to perform the one-sided collective
/// operation for a few iterations and measure average, min and max times across
/// all locations.
///
/// It can be compared against
/// -# @c mpi/collective/osu_gather.c ("--primitive opaque_rmi" flag)
/// -# @c mpi/collective/osu_gatherv.c ("--primitive opaque_rmi" flag)
/// -# @c mpi/collective/osu_reduce.c ("--primitive reduce_rmi" flag)
/// -# @c mpi/collective/osu_bcast.c ("--primitive async_rmi" flag)
/// -# @c openshmem/osu_oshm_broadcast.c ("--primitive async_rmi" flag)
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
#include <boost/utility/in_place_factory.hpp>
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

  // options
  int skip             = SKIP;
  int max_size         = DEFAULT_MAX_SIZE;
  int iterations       = ITERATIONS;
  int iterations_large = ITERATIONS_LARGE;
  bool show_full       = false;
  bool aggregation     = false;
  bool src_diff_gang   = false;
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
      ("src_diff_gang",
       "RMI source location is in a different gang")
      ("primitive",
       po::value<std::string>()->default_value("async_rmi"),
       "Primitive to be used (async_rmi, opaque_rmi, reduce_rmi, "
       "unordered::async_rmi, unordered::opaque_rmi, unordered::reduce_rmi)");

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

    if (vm.count("src_diff_gang"))
      src_diff_gang = true;

    if (vm.count("primitive")) {
      std::string const& s = vm["primitive"].as<std::string>();
      if (s=="async_rmi") {
        pr = rmi_primitive::ASYNC_RMI_ALL_LOCS;
      }
      else if (s=="opaque_rmi") {
        pr = rmi_primitive::OPAQUE_RMI_ALL_LOCS;
      }
      else if (s=="reduce_rmi") {
        pr = rmi_primitive::REDUCE_RMI;
      }
      else if (s=="unordered::async_rmi") {
        pr = rmi_primitive::UNORDERED_ASYNC_RMI_ALL_LOCS;
      }
      else if (s=="unordered::opaque_rmi") {
        pr = rmi_primitive::UNORDERED_OPAQUE_RMI_ALL_LOCS;
      }
      else if (s=="unordered::reduce_rmi") {
        pr = rmi_primitive::UNORDERED_REDUCE_RMI;
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

  // create p_object for data move benchmarks
  unsigned int flags = 0;
  if (!aggregation)
    flags |= no_aggregation;

  distributed_object d_obj{flags, s_buf};

  // create p_object for reduction benchmarks
  typedef std::valarray<float> array_type;
  distributed_value<array_type> d_val{aggregation};

  // create p_object for latency reduction
  distributed_value<double> latencies{true};

  rmi_handle::reference const& r  = d_obj.get_rmi_handle();
  rmi_handle::reference const& rv = d_val.get_rmi_handle();

  const bool sender = (get_location_id()==0);

  if (sender) {
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

  for (int size = 1; size <= max_size; size *= 2) {
    if (size > LARGE_SIZE) {
      skip       = SKIP_LARGE;
      iterations = iterations_large;
    }

    counter<default_timer> counter;
    double time = 0.0;

    rmi_fence(); // quiescence before benchmarking

    if (sender) {
      // if requested, generate traffic from its own gang
      boost::optional<gang> g;
      if (src_diff_gang)
        g = boost::in_place();

      switch (pr) {
        case rmi_primitive::ASYNC_RMI_ALL_LOCS: {
          const auto w = make_range_n(s_buf, size);
          for (int i = 0; i < skip + iterations; i++) {
            counter.reset();
            counter.start();
            async_rmi(all_locations, r,
                      &distributed_object::put_done<decltype(w)>, w);
            d_obj.wait_done();
            const double t = counter.stop();
            if (i>=skip) {
              time += t;
            }
            rmi_fence(); // wait for RMI calls to finish
          }
        } break;
        case rmi_primitive::OPAQUE_RMI_ALL_LOCS:
          for (int i = 0; i < skip + iterations; i++) {
            counter.reset();
            counter.start();
            opaque_rmi(all_locations, r,
                       &distributed_object::get_done, size).get();
            const double t = counter.stop();
            if (i>=skip) {
              time += t;
            }
            rmi_fence(); // wait for RMI calls to finish
          }
          break;
        case rmi_primitive::REDUCE_RMI: {
          if (int(size * sizeof(float)) > max_size) {
            // exit from loop
            size = max_size;
            continue;
          }

          d_val.set_value(array_type(size));
          rmi_fence(); // wait for set_value on all locations

          for (int i = 0; i < skip + iterations; i++) {
            counter.reset();
            counter.start();
            reduce_rmi(std::plus<array_type>{},
                       rv,
                       &distributed_value<array_type>::get_value_done).get();
            const double t = counter.stop();
            if (i>=skip) {
              time += t;
            }
            rmi_fence(); // wait for RMI calls to finish
          }
        } break;
        case rmi_primitive::UNORDERED_ASYNC_RMI_ALL_LOCS: {
          const auto w = make_range_n(s_buf, size);
          for (int i = 0; i < skip + iterations; i++) {
            counter.reset();
            counter.start();
            unordered::async_rmi(all_locations, r,
                                 &distributed_object::put_done<decltype(w)>, w);
            d_obj.wait_done();
            const double t = counter.stop();
            if (i>=skip) {
              time += t;
            }
            rmi_fence(); // wait for RMI calls to finish
          }
        } break;
        case rmi_primitive::UNORDERED_OPAQUE_RMI_ALL_LOCS:
          for (int i = 0; i < skip + iterations; i++) {
            counter.reset();
            counter.start();
            unordered::opaque_rmi(all_locations, r,
                                  &distributed_object::get_done, size).get();
            const double t = counter.stop();
            if (i>=skip) {
              time += t;
            }
            rmi_fence(); // wait for RMI calls to finish
          }
          break;
        case rmi_primitive::UNORDERED_REDUCE_RMI: {
          if (int(size * sizeof(float)) > max_size) {
            // exit from loop
            size = max_size;
            continue;
          }

          d_val.set_value(array_type(size));
          rmi_fence(); // wait for set_value on all locations

          for (int i = 0; i < skip + iterations; i++) {
            counter.reset();
            counter.start();
            unordered::reduce_rmi(
              std::plus<array_type>{},
              rv,
              &distributed_value<array_type>::get_value_done).get();
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

      // finish all traffic from newly created gang
      if (g)
        rmi_fence();
    }
    else {
      switch (pr) {
        case rmi_primitive::ASYNC_RMI_ALL_LOCS:
        case rmi_primitive::OPAQUE_RMI_ALL_LOCS:
        case rmi_primitive::UNORDERED_ASYNC_RMI_ALL_LOCS:
        case rmi_primitive::UNORDERED_OPAQUE_RMI_ALL_LOCS:
          for (int i = 0; i < skip + iterations; i++) {
            counter.reset();
            counter.start();
            d_obj.wait_done();
            const double t = counter.stop();
            if (i>=skip) {
              time += t;
            }
            rmi_fence(); // wait for RMI calls to finish
          }
          break;
        case rmi_primitive::REDUCE_RMI:
        case rmi_primitive::UNORDERED_REDUCE_RMI: {
          if (int(size * sizeof(float)) > max_size) {
            // exit from loop
            size = max_size;
            continue;
          }

          d_val.set_value(array_type(size));
          rmi_fence(); // wait for set_value on all locations

          for (int i = 0; i < skip + iterations; i++) {
            counter.reset();
            counter.start();
            d_val.wait_done();
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
    }

    const double latency = ((time * 1e6) / double(iterations));
    latencies.set_value(latency);
    rmi_fence(); // wait for set_value on all locations

    if (sender) {
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
  }

  rmi_fence(); // wait for RMI calls to finish

  delete[] buf;

  return EXIT_SUCCESS;
}
