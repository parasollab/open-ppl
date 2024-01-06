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
/// Benchmark for @ref stapl::async_rmi() / @ref stapl::try_rmi() bandwidth.
///
/// The main idea of the benchmark is to send a number of RMIs with a specific
/// payload size and measure the time that it took for all of them to be
/// processed.
///
/// It can be compared against
/// -# @c mpi/one-sided/osu_put_bw.c
/// -# @c mpi/pt2pt/osu_bw.c
/// in OSU benchmarks.
///
/// The bidirectional benchmark can be compared against
/// -# @c mpi/one-sided/osu_put_bibw.c
/// -# @c mpi/pt2pt/osu_bibw.c
/// in OSU benchmarks.
//////////////////////////////////////////////////////////////////////

#define BENCHMARK       "# %s Bandwidth Benchmark\n"
#define BENCHMARK_BIDIR "# %s Bi-directional Bandwidth Benchmark\n"

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


// Prints the achieved bandwidth
void print(int size, int loop, int window_size, double t)
{
  const double tmp = size / 1e6 * loop * window_size;
  std::fprintf(stdout, "%-*d%*.*f\n",
               10, size, FIELD_WIDTH, FLOAT_PRECISION, (tmp / t));
  std::fflush(stdout);
}


// async_rmi() with one-sided synchronization (async_rmi() or sync_rmi())
void async_rmi_os(const int skip, const int loop, const bool bidirectional,
                  const bool src_diff_gang, const bool deferred_sync,
                  distributed_object& d_obj,
                  const int window_size, const std::size_t size)
{
  const unsigned int lid = get_location_id();

  if (!bidirectional && lid!=0)
    return;

  const unsigned int peer_lid    = d_obj.get_peer();
  rmi_handle::reference const& r = d_obj.get_rmi_handle();
  char* const s_buf              = d_obj.get_data_ptr();

  counter<default_timer> counter;

  // if requested, generate traffic from its own gang
  boost::optional<gang> g;
  if (src_diff_gang)
    g = boost::in_place();

  touch_data(s_buf, (size * window_size));

  if (deferred_sync) {
    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      for (int j = 0; j < window_size; j++) {
        const auto w = make_range_n(s_buf + (j*size), size);
        async_rmi(peer_lid, r, &distributed_object::put_ack<decltype(w)>, w);
      }
      d_obj.wait_done();
    }
  }
  else {
    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      for (int j = 0; j < window_size; j++) {
        const auto w = make_range_n(s_buf + (j*size), size);
        async_rmi(peer_lid, r, &distributed_object::put<decltype(w)>, w);
      }
      sync_rmi(peer_lid, r, &distributed_object::recv_sync);
    }
  }
  const double t = counter.stop();

  // finish all traffic from newly created gang
  if (g)
    rmi_fence();

  if (lid == 0)
    print(size, loop, window_size, t);
}


// async_rmi() with rmi_fence() synchronization
void async_rmi_fence(const int skip, const int loop, const bool bidirectional,
                     const bool src_diff_gang,
                     distributed_object& d_obj,
                     const int window_size, const std::size_t size)
{
  const unsigned int peer_lid    = d_obj.get_peer();
  rmi_handle::reference const& r = d_obj.get_rmi_handle();
  char* const s_buf              = d_obj.get_data_ptr();

  if (get_location_id() == 0) {
    counter<default_timer> counter;

    // if requested, generate traffic from its own gang
    boost::optional<gang> g;
    if (src_diff_gang)
      g = boost::in_place();

    touch_data(s_buf, (size * window_size));

    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      for (int j = 0; j < window_size; j++) {
        const auto w = make_range_n(s_buf + (j*size), size);
        async_rmi(peer_lid, r, &distributed_object::put<decltype(w)>, w);
      }
      rmi_fence(); // wait for async_rmi calls to finish
    }
    const double t = counter.stop();

    print(size, loop, window_size, t);
  }
  else {
    for (int i = 0; i < skip + loop; i++) {
      // if requested, generate traffic from its own gang
      boost::optional<gang> g;
      if (src_diff_gang)
        g = boost::in_place();

      if (bidirectional) {
        for (int j = 0; j < window_size; j++) {
        const auto w = make_range_n(s_buf + (j*size), size);
          async_rmi(peer_lid, r, &distributed_object::put<decltype(w)>, w);
        }
      }

      rmi_fence(); // wait for async_rmi calls to finish
    }
  }
}


// try_rmi() with one-sided synchronization (async_rmi() or sync_rmi())
void try_rmi_os(const int skip, const int loop, const bool bidirectional,
                const bool src_diff_gang, const bool deferred_sync,
                distributed_object& d_obj,
                const int window_size, const std::size_t size)
{
  const unsigned int lid = get_location_id();

  if (!bidirectional && lid!=0)
    return;

  const unsigned int peer_lid    = d_obj.get_peer();
  rmi_handle::reference const& r = d_obj.get_rmi_handle();
  char* const s_buf              = d_obj.get_data_ptr();

  counter<default_timer> counter;

  // if requested, generate traffic from its own gang
  boost::optional<gang> g;
  if (src_diff_gang)
    g = boost::in_place();

  touch_data(s_buf, (size * window_size));

  if (deferred_sync) {
    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      for (int j = 0; j < window_size; j++) {
        const auto w = make_range_n(s_buf + (j*size), size);
        try_rmi(peer_lid, r, &distributed_object::put_ack<decltype(w)>, w);
      }
      d_obj.wait_done();
    }
  }
  else {
    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      for (int j = 0; j < window_size; j++) {
        const auto w = make_range_n(s_buf + (j*size), size);
        try_rmi(peer_lid, r, &distributed_object::put<decltype(w)>, w);
      }
      sync_rmi(peer_lid, r, &distributed_object::recv_sync);
    }
  }
  const double t = counter.stop();

  // finish all traffic from newly created gang
  if (g)
    rmi_fence();

  if (lid==0)
    print(size, loop, window_size, t);
}


// try_rmi() with rmi_fence() synchronization
void try_rmi_fence(const int skip, const int loop, const bool bidirectional,
                   const bool src_diff_gang,
                   distributed_object& d_obj,
                   const int window_size, const std::size_t size)
{
  const unsigned int peer_lid    = d_obj.get_peer();
  rmi_handle::reference const& r = d_obj.get_rmi_handle();
  char* const s_buf              = d_obj.get_data_ptr();

  if (get_location_id() == 0) {
    counter<default_timer> counter;

    // if requested, generate traffic from its own gang
    boost::optional<gang> g;
    if (src_diff_gang)
      g = boost::in_place();

    touch_data(s_buf, (size * window_size));

    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      for (int j = 0; j < window_size; j++) {
        const auto w = make_range_n(s_buf + (j*size), size);
        try_rmi(peer_lid, r, &distributed_object::put<decltype(w)>, w);
      }
      rmi_fence(); // wait for async_rmi calls to finish
    }
    const double t = counter.stop();

    print(size, loop, window_size, t);
  }
  else {
    for (int i = 0; i < skip + loop; i++) {
      // if requested, generate traffic from its own gang
      boost::optional<gang> g;
      if (src_diff_gang)
        g = boost::in_place();

      if (bidirectional) {
        for (int j = 0; j < window_size; j++) {
          const auto w = make_range_n(s_buf + (j*size), size);
          try_rmi(peer_lid, r, &distributed_object::put<decltype(w)>, w);
        }
      }

      rmi_fence(); // wait for async_rmi calls to finish
    }
  }
}


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
  rmi_primitive spr  = rmi_primitive::UNKNOWN;
  bool bidirectional = false;

  // parse options
  try {
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
      ("help",
       "Print this help message")
      ("no_aggregation", "Disable aggregation")
      ("src_diff_gang",
       "RMI source location is in a different gang")
      ("primitive",
       po::value<std::string>()->default_value("async_rmi"),
       "Primitive to be used (async_rmi, try_rmi)")
      ("sync",
       po::value<std::string>()->default_value("async_rmi"),
       "Primitive to be used for synchronization (async_rmi, sync_rmi, "
       "rmi_fence)")
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
      if (s=="async_rmi") {
        pr = rmi_primitive::ASYNC_RMI;
      }
      else if (s=="try_rmi") {
        pr = rmi_primitive::TRY_RMI;
      }
      else {
        if (get_location_id()==0)
          error("%s: Incorrect primitive.\n", argv[0]);
        return EXIT_FAILURE;
      }
    }

    if (vm.count("sync")) {
      std::string const& s = vm["sync"].as<std::string>();
      if (s=="async_rmi") {
        spr = rmi_primitive::ASYNC_RMI;
      }
      else if (s=="sync_rmi") {
        spr = rmi_primitive::SYNC_RMI;
      }
      else if (s=="rmi_fence") {
        spr = rmi_primitive::FENCE;
      }
      else {
        if (get_location_id()==0)
          error("%s: Incorrect sync primitive.\n", argv[0]);
        return EXIT_FAILURE;
      }
    }

    if (vm.count("bidirectional"))
      bidirectional = true;
  }
  catch (boost::program_options::unknown_option& ex) {
    error("%s: %s.\n", argv[0], ex.what());
  }

  // allocate payload buffer
  const int page_size = getpagesize();
  assert(page_size <= MAX_ALIGNMENT);
  char* buf   = new char[(MAX_SIZE * window_size + MAX_ALIGNMENT)];
  char* s_buf = static_cast<char*>(align_buffer(buf, page_size));

  // create p_object
    unsigned int flags = 0;
  if (!aggregation)
    flags |= no_aggregation;
  if (pr == rmi_primitive::TRY_RMI)
    flags |= allow_try_rmi;

  distributed_object d_obj{flags, s_buf};

  if (get_location_id() == 0) {
    if (!bidirectional)
      std::fprintf(stdout, BENCHMARK, get_primitive_name(pr));
    else
      std::fprintf(stdout, BENCHMARK_BIDIR, get_primitive_name(pr));
    std::fprintf(stdout, "# Synchronization: %s\n", get_primitive_name(spr));
    std::fprintf(stdout, "# Aggregation: %s\n", (aggregation ? "yes" : "no"));
    std::fprintf(stdout, "# Intragang: %s\n", (src_diff_gang ? "no" : "yes"));
    std::fprintf(stdout, "%-*s%*s\n",
                 10, "# Size", FIELD_WIDTH, "Bandwidth (MB/s)");
    std::fflush(stdout);
  }

  // bandwidth benchmark
  for (int size = 1; size <= MAX_SIZE; size *= 2) {
    if (size > LARGE_MESSAGE_SIZE) {
      loop = LOOP_LARGE;
      skip = SKIP_LARGE;
    }
    d_obj.set_counter(window_size);

    rmi_fence(); // quiescence before benchmarking

    switch (pr) {
      case rmi_primitive::ASYNC_RMI:
        switch (spr) {
          case rmi_primitive::ASYNC_RMI:
            async_rmi_os(skip, loop, bidirectional,
                         src_diff_gang, true,
                         d_obj, window_size, size);
            break;
          case rmi_primitive::SYNC_RMI:
            async_rmi_os(skip, loop, bidirectional,
                         src_diff_gang, false,
                         d_obj, window_size, size);
            break;
          case rmi_primitive::FENCE:
            async_rmi_fence(skip, loop, bidirectional,
                            src_diff_gang,
                            d_obj, window_size, size);
            break;
          default:
            error("%s: Incorrect sync primitive.\n", argv[0]);
            break;
        }
        break;
      case rmi_primitive::TRY_RMI:
        switch (spr) {
          case rmi_primitive::ASYNC_RMI:
            try_rmi_os(skip, loop, bidirectional,
                       src_diff_gang, true,
                       d_obj, window_size, size);
            break;
          case rmi_primitive::SYNC_RMI:
            try_rmi_os(skip, loop, bidirectional,
                       src_diff_gang, false,
                       d_obj, window_size, size);
            break;
          case rmi_primitive::FENCE:
            try_rmi_fence(skip, loop, bidirectional,
                          src_diff_gang,
                          d_obj, window_size, size);
            break;
          default:
            error("%s: Incorrect sync primitive.\n", argv[0]);
            break;
        }
        break;
      default:
        error("%s: Incorrect primitive.\n", argv[0]);
        break;
    }

    rmi_fence(); // wait for async_rmi calls to finish
  }

  delete[] buf;

  return EXIT_SUCCESS;
}
