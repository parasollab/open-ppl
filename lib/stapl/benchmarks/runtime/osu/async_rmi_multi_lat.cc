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
/// Benchmark for multi-pair latency using @ref stapl::async_rmi().
///
/// The main idea of the benchmark is the for each pair of locations, one is
/// sending and receiving a buffer of data and the other is processing the
/// request, and then the roles reverse.
///
/// It can be compared against
/// -# @c mpi/pt2pt/osu_multi_lat.c
/// in OSU benchmarks.
//////////////////////////////////////////////////////////////////////

#define BENCHMARK "# %s Multi Latency Benchmark\n"

#include <stapl/runtime.hpp>
#include <cstdio>
#include <functional>
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

#define LARGE_MESSAGE_SIZE 8192

#define LOOP_SMALL  10000
#define SKIP_SMALL  100
#define LOOP_LARGE  1000
#define SKIP_LARGE  10

#ifndef FIELD_WIDTH
# define FIELD_WIDTH 20
#endif

#ifndef FLOAT_PRECISION
# define FLOAT_PRECISION 2
#endif


using namespace stapl;


exit_code stapl_main(int argc, char *argv[])
{
  int loop = LOOP_SMALL;
  int skip = SKIP_SMALL;

  // options
  bool aggregation   = false;
  bool src_diff_gang = false;
  bool use_stack     = false;

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
      ("use_stack",
       "If defined, then the buffer is created on the stack.");

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

    if (vm.count("use_stack"))
      use_stack = true;
  }
  catch (boost::program_options::unknown_option& ex) {
    error("%s: %s.\n", argv[0], ex.what());
  }

  if (get_num_locations() % 2 != 0) {
    if (get_location_id() == 0)
      error("%s: This benchmark requires even number of locations.\n", argv[0]);
    return EXIT_FAILURE;
  }

  const unsigned int pairs = (get_num_locations() / 2);
  const unsigned int lid   = get_location_id();

  char stack_buf[BUFFER_SIZE];
  char* buf   = nullptr;
  char* s_buf = nullptr;
  const int page_size = getpagesize();
  assert(page_size <= MAX_ALIGNMENT);

  if (!use_stack) {
    buf   = new char[BUFFER_SIZE];
    s_buf = static_cast<char*>(align_buffer(buf, page_size));
  }
  else {
    s_buf = static_cast<char*>(align_buffer(stack_buf, page_size));
  }

  // create p_object
  unsigned int flags = 0;
  if (!aggregation)
    flags |= no_aggregation;

  distributed_object d_obj{flags, s_buf};

  if (lid == 0) {
    std::fprintf(stdout, BENCHMARK,
                 get_primitive_name(rmi_primitive::ASYNC_RMI));
    std::fprintf(stdout, "# Aggregation: %s\n", (aggregation ? "yes" : "no"));
    std::fprintf(stdout, "# Intragang: %s\n", (src_diff_gang ? "no" : "yes"));
    std::fprintf(stdout, "# Pairs: %u\n", pairs);
    std::fprintf(stdout, "# Locations: %u\n", get_num_locations());
    std::fprintf(stdout, "%-*s%*s\n",
                 10, "# Size", FIELD_WIDTH, "Latency (us)");
    std::fflush(stdout);
  }

  rmi_handle::reference const& r = d_obj.get_rmi_handle();
  const unsigned int peer_lid = ((lid < pairs) ? lid + pairs
                                               : lid - pairs);

  // latency benchmark
  for (int size = 0; size <= MAX_SIZE; size = (size ? size * 2 : size + 1)) {
    if (size > LARGE_MESSAGE_SIZE) {
      loop = LOOP_LARGE;
      skip = SKIP_LARGE;
    }

    counter<default_timer> counter;

    const auto w = make_range_n(s_buf, size);

    touch_data(s_buf, size);

    rmi_fence(); // quiescence before benchmarking

    // if requested, generate traffic from its own gang
    boost::optional<gang> g;
    if (src_diff_gang)
      g = boost::in_place();

    if (lid < pairs) {
      for (int i = 0; i < skip + loop; i++) {
        if (i == skip) {
          counter.start();
          rmi_barrier();
        }

        async_rmi(peer_lid, r, &distributed_object::put_done<decltype(w)>, w);
        d_obj.wait_done();
      }
    }
    else {
      for (int i = 0; i < skip + loop; i++) {
        if (i == skip) {
          counter.start();
          rmi_barrier();
        }

        d_obj.wait_done();
        async_rmi(peer_lid, r, &distributed_object::put_done<decltype(w)>, w);
      }
    }
    const double t       = counter.stop();
    const double latency = (t * 1.0e6 / (2.0 * loop));

    // finish all traffic from newly created gang
    if (g) {
      rmi_fence();
      g->leave();
    }

    const double total_latency = reduce_value(std::plus<double>(), latency);
    if (lid == 0) {
      const double avg_latency = (total_latency / (2.0 * pairs));
      std::fprintf(stdout, "%-*d%*.*f\n",
                   10, size, FIELD_WIDTH, FLOAT_PRECISION, avg_latency);
      std::fflush(stdout);
    }

    rmi_fence(); // wait for async_rmi calls to finish
  }

  delete[] buf;

  return EXIT_SUCCESS;
}
