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
/// @ref stapl::promise latency.
///
/// The main idea of the benchmark is to get data by sending a number of RMIs
/// with a specific required size for the returned data and measure the time
/// that it had taken for them to be processed.
///
/// It can be compared against
/// -# @c mpi/one-sided/osu_get_latency.c
/// -# @c openshmem/osu_oshm_get.c
/// in OSU benchmarks.
///
/// It can also be compared against
/// -# @c upc/osu_upc_memget.c
/// in OSU benchmarks when the flag "-use_pairs" is used.
//////////////////////////////////////////////////////////////////////

#define BENCHMARK "# %s Latency Benchmark\n"

#include <stapl/runtime.hpp>
#include <cstdio>
#include <iostream>
#include <string>
#include <unistd.h>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/utility/in_place_factory.hpp>
#include "common.hpp"

#define MAX_ALIGNMENT 65536
#define MAX_SIZE      (1<<22)
#define BUFFER_SIZE   (MAX_SIZE + MAX_ALIGNMENT)

#define SKIP_LARGE         10
#define LOOP_LARGE         100
#define LARGE_MESSAGE_SIZE 8192

#ifndef FIELD_WIDTH
# define FIELD_WIDTH 20
#endif

#ifndef FLOAT_PRECISION
# define FLOAT_PRECISION 2
#endif


using namespace stapl;


exit_code stapl_main(int argc, char *argv[])
{
  int skip = 1000;
  int loop = 10000;

  // options
  bool aggregation   = false;
  bool src_diff_gang = false;
  rmi_primitive pr   = rmi_primitive::UNKNOWN;
  bool use_stack     = false;
  bool use_pairs     = false;

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
       "Primitive to be used (sync_rmi, opaque_rmi, promise)")
      ("use_stack",
       "If defined, then the buffer is created on the stack.")
      ("use_pairs",
       "If defined, half of the locations are receiving elements from the "
       "other half.");

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

    if (vm.count("use_stack"))
      use_stack = true;

    if (vm.count("use_pairs"))
      use_pairs = true;
  }
  catch (boost::program_options::unknown_option& ex) {
    error("%s: %s.\n", argv[0], ex.what());
  }

  if (use_pairs && src_diff_gang) {
    if (get_location_id() == 0)
      error("%s: --use_pairs is not allowed when src_diff_gang is defined.\n",
            argv[0]);
    return EXIT_FAILURE;
  }

  if (!use_pairs) {
    if (get_num_locations() != 2) {
      if (get_location_id() == 0)
        error("%s: This benchmark requires two locations.\n", argv[0]);
      return EXIT_FAILURE;
    }
  }
  else {
    if (get_num_locations() % 2 != 0) {
      if (get_location_id() == 0)
        error("%s: This benchmark requires even number of locations.\n",
              argv[0]);
      return EXIT_FAILURE;
    }
  }

  // even numbered locations are receiving elements
  const bool getter = (get_location_id() % 2 == 0);
  const bool printer = (get_location_id()==0);

  // create p_object
  unsigned int flags = 0;
  if (!aggregation)
    flags |= no_aggregation;

  distributed_object d_obj(flags);

  char stack_buf[BUFFER_SIZE];
  char* buf   = nullptr;
  char* s_buf = nullptr;
  if (!getter) {
    const int page_size = getpagesize();
    assert(page_size <= MAX_ALIGNMENT);

    if (!use_stack) {
      buf   = new char[BUFFER_SIZE];
      s_buf = static_cast<char*>(align_buffer(buf, page_size));
    }
    else {
      s_buf = static_cast<char*>(align_buffer(stack_buf, page_size));
    }

    // Set pointer to data
    d_obj.set_data_ptr(s_buf);
  }

  if (printer) {
    std::fprintf(stdout, BENCHMARK, get_primitive_name(pr));
    std::fprintf(stdout, "# Aggregation: %s\n", (aggregation ? "yes" : "no"));
    std::fprintf(stdout, "# Intragang: %s\n", (src_diff_gang ? "no" : "yes"));
    if (use_pairs)
      std::fprintf(stdout, "# Pairs: %d\n", (get_num_locations() / 2));
    std::fprintf(stdout, "%-*s%*s\n",
                 10, "# Size", FIELD_WIDTH, "Latency (us)");
    std::fflush(stdout);
  }

  rmi_handle::reference const& r = d_obj.get_rmi_handle();
  const unsigned int peer_lid = ((get_location_id()+1) % get_num_locations());

  // latency benchmark
  for (int size = 0; size <= MAX_SIZE; size = (size ? size * 2 : size + 1)) {
    if (size > LARGE_MESSAGE_SIZE) {
      loop = LOOP_LARGE;
      skip = SKIP_LARGE;
    }

    counter<default_timer> counter;

    if (!getter) {
      touch_data(s_buf, size);
    }

    rmi_fence(); // quiescence before benchmarking

    if (getter) {
      // if requested, generate traffic from its own gang
      boost::optional<gang> g;
      if (src_diff_gang)
        g = boost::in_place();

      switch (pr) {
        case rmi_primitive::SYNC_RMI:
          for (int i = 0; i < skip + loop; i++) {
            if (i == skip)
              counter.start();

            sync_rmi(peer_lid, r, &distributed_object::get_nodisp, size);
          }
          break;
        case rmi_primitive::OPAQUE_RMI:
          for (int i = 0; i < skip + loop; i++) {
            if (i == skip)
              counter.start();

            future<std::vector<char>> f =
              opaque_rmi(peer_lid, r, &distributed_object::get_nodisp, size);
            f.get();
          }
          break;
        case rmi_primitive::PROMISE_WITH_ASYNC_RMI:
          for (int i = 0; i < skip + loop; i++) {
            if (i == skip)
              counter.start();

            promise<std::vector<char>> p;
            auto f = p.get_future();
            async_rmi(peer_lid, r, &distributed_object::get_nodisp_promise,
                      std::move(p), size);
            f.get();
          }
          break;
        default:
          error("%s: Incorrect primitive.\n", argv[0]);
          break;
      }

      if (use_pairs) {
        // when using pairs, a barrier is needed to ensure that all locations
        // have finished
        rmi_barrier();
      }
      const double t = counter.stop();

      if (printer) {
        const double latency = (t * 1.0e6 / loop);
        std::fprintf(stdout, "%-*d%*.*f\n",
                     10, size, FIELD_WIDTH, FLOAT_PRECISION, latency);
        std::fflush(stdout);
      }

      // finish all traffic from newly created gang
      if (g)
        rmi_fence();
    }
    else if (use_pairs) {
      rmi_barrier();
    }

    rmi_fence(); // wait for RMI calls to finish
  }

  delete[] buf;

  return EXIT_SUCCESS;
}
