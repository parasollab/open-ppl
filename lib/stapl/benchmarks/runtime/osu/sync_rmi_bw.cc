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
/// @ref stapl::promise bandwidth.
///
/// The main idea of the benchmark is to get data by sending a number of RMIs
/// with a specific required size for the returned data and measure the time
/// that it had taken for them to be processed.
///
/// It can be compared against
/// -# @c mpi/one-sided/osu_get_bw.c
/// in OSU benchmarks.
//////////////////////////////////////////////////////////////////////

#define BENCHMARK       "# %s Bandwidth Benchmark\n"
#define BENCHMARK_BIDIR "# %s Bi-directional Bandwidth Benchmark\n"

#include <stapl/runtime.hpp>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/utility/in_place_factory.hpp>
#include "common.hpp"

#define MAX_ALIGNMENT 65536
#define MAX_SIZE      (1<<22)

#define LOOP_LARGE         30
#define SKIP_LARGE         10
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
  if (get_num_locations() != 2) {
    if (get_location_id() == 0)
      error("%s: This benchmark requires two locations.\n", argv[0]);
    return EXIT_FAILURE;
  }

  const int window_size = 32;
  int loop              = 100;
  int skip              = 20;

  // options
  bool aggregation   = true;
  bool src_diff_gang = false;
  rmi_primitive pr   = rmi_primitive::UNKNOWN;
  bool bidirectional = false;

  // parse options
  try {
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
      ("help",
       "Print this help message")
      ("no_aggregation",
       "Disable aggregation")
      ("src_diff_gang",
       "RMI source location is in a different gang")
      ("primitive",
       po::value<std::string>()->default_value("sync_rmi"),
       "Primitive to be used (sync_rmi, opaque_rmi, promise)")
      ("bidirectional",
       "Do bidirectional bandwidth benchmark");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      if (get_location_id()==0)
        std::cout << desc << std::endl;
      return EXIT_SUCCESS;
    }

    if (vm.count("no_aggregation"))
      aggregation = false;

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

    if (vm.count("bidirectional"))
      bidirectional = true;
  }
  catch (boost::program_options::unknown_option& ex) {
    error("%s: %s.\n", argv[0], ex.what());
  }

  const unsigned int lid      = get_location_id();
  const unsigned int peer_lid = ((get_location_id() + 1) % get_num_locations());

  // create p_object
  unsigned int flags = 0;
  if (!aggregation)
    flags |= no_aggregation;

  distributed_object d_obj(flags);

  // allocate payload buffer
  char* buf   = nullptr;
  char* s_buf = nullptr;
  if (bidirectional || lid == 1) {
    const std::size_t buf_size = (MAX_SIZE * window_size + MAX_ALIGNMENT);

    const int page_size = getpagesize();
    assert(page_size <= MAX_ALIGNMENT);

    buf   = new char[buf_size];
    s_buf = static_cast<char*>(align_buffer(buf, page_size));

    // Set pointer to data
    d_obj.set_data_ptr(s_buf);
  }

  if (lid == 0) {
    if (!bidirectional)
      std::fprintf(stdout, BENCHMARK, get_primitive_name(pr));
    else
      std::fprintf(stdout, BENCHMARK_BIDIR, get_primitive_name(pr));
    std::fprintf(stdout, "# Aggregation: %s\n", (aggregation ? "yes" : "no"));
    std::fprintf(stdout, "# Intragang: %s\n", (src_diff_gang ? "no" : "yes"));
    std::fprintf(stdout, "%-*s%*s\n",
                 10, "# Size", FIELD_WIDTH, "Bandwidth (MB/s)");
    std::fflush(stdout);
  }

  rmi_handle::reference const& r = d_obj.get_rmi_handle();

  // bandwidth benchmark
  for (int size = 1; size <= MAX_SIZE; size *= 2) {
    if (size > LARGE_MESSAGE_SIZE) {
      loop = LOOP_LARGE;
      skip = SKIP_LARGE;
    }

    counter<default_timer> counter;

    if (bidirectional || lid == 1)
      touch_data(s_buf, size * window_size);

    rmi_fence(); // quiescence before benchmarking

    // if requested, generate traffic from its own gang
    boost::optional<gang> g;
    if (src_diff_gang)
      g = boost::in_place();

    switch (pr) {
      case rmi_primitive::SYNC_RMI:
        if (bidirectional || lid==0) {
          for (int i = 0; i < skip + loop; i++) {
            if (i == skip)
              counter.start();

            for (int j = 0; j < window_size; j++) {
              sync_rmi(peer_lid, r, &distributed_object::get,
                       (j*size), size);
            }
          }
        }
        break;
      case rmi_primitive::OPAQUE_RMI:
        if (bidirectional || lid==0) {
          std::vector<future<std::vector<char>>> vf(window_size);
          for (int i = 0; i < skip + loop; i++) {
            if (i == skip)
              counter.start();

            for (int j = 0; j < window_size; j++) {
              vf[j] = opaque_rmi(peer_lid, r, &distributed_object::get,
                                 (j*size), size);
            }
            for (int j = 0; j < window_size; j++) {
              vf[j].get();
            }
          }
        }
        break;
      case rmi_primitive::PROMISE_WITH_ASYNC_RMI:
        if (bidirectional || lid==0) {
          std::vector<future<std::vector<char>>> f;
          f.reserve(window_size);
          for (int i = 0; i < skip + loop; i++) {
            if (i == skip)
              counter.start();

            for (int j = 0; j < window_size; j++) {
              promise<std::vector<char>> p;
              f.emplace_back(p.get_future());
              async_rmi(peer_lid, r, &distributed_object::get_promise,
                        std::move(p), (j*size), size);
            }
            for (int j = 0; j < window_size; j++) {
              f[j].get();
            }
            f.clear();
          }
        }
        break;
      default:
        error("%s: Incorrect primitive.\n", argv[0]);
        break;
    }

    if (bidirectional || lid == 0) {
      const double t = counter.stop();
      const double tmp = size / 1e6 * loop * window_size;

      if (lid==0) {
        std::fprintf(stdout, "%-*d%*.*f\n",
                     10, size, FIELD_WIDTH, FLOAT_PRECISION, (tmp / t));
        std::fflush(stdout);
      }
    }

    // finish all traffic from newly created gang
    if (g) {
      rmi_fence();
      g->leave();
    }

    rmi_fence(); // wait for RMI calls to finish
  }

  delete[] buf;

  return EXIT_SUCCESS;
}
