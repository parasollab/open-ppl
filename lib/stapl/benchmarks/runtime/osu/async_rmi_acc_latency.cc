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
/// Benchmark for @ref stapl::async_rmi() / @ref stapl::try_rmi() latency for
/// remote accumulation.
///
/// The main idea of the benchmark is to send a number of RMIs with a specific
/// payload size and measure the time that it took to add the payload of the RMI
/// to the remote buffer.
///
/// It can be compared against
/// -# @c mpi/one-sided/osu_acc_latency.c
/// in OSU benchmarks.
//////////////////////////////////////////////////////////////////////

#define BENCHMARK "# %s Accumulate Latency Benchmark\n"

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


// Prints the achieved latency
void print(int size, int loop, double t, bool ping_pong)
{
  const double latency = (ping_pong ? (t * 1.0e6 / loop / 2.0)
                                    : (t * 1.0e6 / loop));
  std::fprintf(stdout, "%-*d%*.*f\n",
               10, size, FIELD_WIDTH, FLOAT_PRECISION, latency);
  std::fflush(stdout);
}


// async_rmi() with one-sided synchronization (async_rmi() or sync_rmi())
void async_rmi_os(const int skip, const int loop,
                  const bool src_diff_gang, const bool deferred_sync,
                  distributed_object& d_obj, const std::size_t size)
{
  if (get_location_id() != 0)
    return;

  rmi_handle::reference const& r = d_obj.get_rmi_handle();
  char* const s_buf              = d_obj.get_data_ptr();
  const auto w                   = make_range_n(s_buf, size);

  counter<default_timer> counter;

  // if requested, generate traffic from its own gang
  boost::optional<gang> g;
  if (src_diff_gang)
    g = boost::in_place();

  touch_data(s_buf, size);

  if (deferred_sync) {
    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      async_rmi(1, r, &distributed_object::add_ack<decltype(w)>, w);
    }
    d_obj.wait_done();
  }
  else {
    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      async_rmi(1, r, &distributed_object::add<decltype(w)>, w);
    }
    sync_rmi(1, r, &distributed_object::recv_sync);
  }
  const double t = counter.stop();

  // finish all traffic from newly created gang
  if (g)
    rmi_fence();

  print(size, loop, t, false);
}


// async_rmi() with rmi_fence() synchronization
void async_rmi_fence(const int skip, const int loop,
                     const bool src_diff_gang,
                     distributed_object& d_obj, const std::size_t size)
{
  if (get_location_id() == 0) {
    rmi_handle::reference const& r = d_obj.get_rmi_handle();
    char* const s_buf              = d_obj.get_data_ptr();
    const auto w                   = make_range_n(s_buf, size);

    counter<default_timer> counter;

    // if requested, generate traffic from its own gang
    boost::optional<gang> g;
    if (src_diff_gang)
      g = boost::in_place();

    touch_data(s_buf, size);

    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      async_rmi(1, r, &distributed_object::add<decltype(w)>, w);
    }
    rmi_fence(); // wait for async_rmi calls to finish
    const double t = counter.stop();

    print(size, loop, t, false);
  }
  else if (!src_diff_gang) {
    rmi_fence(); // wait for async_rmi calls to finish
  }
}


// async_rmi() ping-pong
void async_rmi_pp(const int skip, const int loop,
                  const bool src_diff_gang,
                  distributed_object& d_obj, const std::size_t size)
{
  if (get_location_id() != 0)
    return;

  rmi_handle::reference const& r = d_obj.get_rmi_handle();
  char* const s_buf              = d_obj.get_data_ptr();
  const auto w                   = make_range_n(s_buf, size);

  counter<default_timer> counter;

  // if requested, generate traffic from its own gang
  boost::optional<gang> g;
  if (src_diff_gang)
    g = boost::in_place();

  touch_data(s_buf, size);

  for (int i = 0; i < skip + loop; i++) {
    if (i == skip)
      counter.start();

    async_rmi(1, r, &distributed_object::add_ping<decltype(w)>, w);
    d_obj.wait_done();
  }
  const double t = counter.stop();

  // finish all traffic from newly created gang
  if (g)
    rmi_fence();

  print(size, loop, t, true);
}


// try_rmi() with one-sided synchronization (async_rmi() or sync_rmi())
void try_rmi_os(const int skip, const int loop,
                const bool src_diff_gang, const bool deferred_sync,
                distributed_object& d_obj, const std::size_t size)
{
  if (get_location_id() != 0)
    return;

  rmi_handle::reference const& r = d_obj.get_rmi_handle();
  char* const s_buf              = d_obj.get_data_ptr();
  const auto w                   = make_range_n(s_buf, size);

  counter<default_timer> counter;

  // if requested, generate traffic from its own gang
  boost::optional<gang> g;
  if (src_diff_gang)
    g = boost::in_place();

  touch_data(s_buf, size);

  if (deferred_sync) {
    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      try_rmi(1, r, &distributed_object::add_ack<decltype(w)>, w);
    }
    d_obj.wait_done();
  }
  else {
    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      try_rmi(1, r, &distributed_object::add<decltype(w)>, w);
    }
    sync_rmi(1, r, &distributed_object::recv_sync);
  }
  const double t = counter.stop();

  // finish all traffic from newly created gang
  if (g)
    rmi_fence();

  print(size, loop, t, false);
}


// try_rmi() with rmi_fence() synchronization
void try_rmi_fence(const int skip, const int loop,
                   const bool src_diff_gang,
                   distributed_object& d_obj, const std::size_t size)
{
  if (get_location_id() == 0) {
    rmi_handle::reference const& r = d_obj.get_rmi_handle();
    char* const s_buf              = d_obj.get_data_ptr();
    const auto w                   = make_range_n(s_buf, size);

    counter<default_timer> counter;

    // if requested, generate traffic from its own gang
    boost::optional<gang> g;
    if (src_diff_gang)
      g = boost::in_place();

    touch_data(s_buf, size);

    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      try_rmi(1, r, &distributed_object::add<decltype(w)>, w);
    }
    rmi_fence(); // wait for async_rmi calls to finish
    const double t = counter.stop();

    print(size, loop, t, false);
  }
  else if (!src_diff_gang) {
    rmi_fence(); // wait for async_rmi calls to finish
  }
}


// try_rmi() ping-pong
void try_rmi_pp(const int skip, const int loop,
                const bool src_diff_gang,
                distributed_object& d_obj, const std::size_t size)
{
  if (get_location_id() != 0)
    return;

  rmi_handle::reference const& r = d_obj.get_rmi_handle();
  char* const s_buf              = d_obj.get_data_ptr();
  const auto w                   = make_range_n(s_buf, size);

  counter<default_timer> counter;

  // if requested, generate traffic from its own gang
  boost::optional<gang> g;
  if (src_diff_gang)
    g = boost::in_place();

  touch_data(s_buf, size);

  for (int i = 0; i < skip + loop; i++) {
    if (i == skip)
      counter.start();

    try_rmi(1, r, &distributed_object::add_ping<decltype(w)>, w);
    d_obj.wait_done();
  }
  const double t = counter.stop();

  // finish all traffic from newly created gang
  if (g)
    rmi_fence();

  print(size, loop, t, true);
}


exit_code stapl_main(int argc, char *argv[])
{
  if (get_num_locations() != 2) {
    if (get_location_id()==0)
      error("%s: This benchmark requires two locations.\n", argv[0]);
    return EXIT_FAILURE;
  }

  int skip = 1000;
  int loop = 10000;

  // options
  bool aggregation   = false;
  bool src_diff_gang = false;
  rmi_primitive pr   = rmi_primitive::UNKNOWN;
  rmi_primitive spr  = rmi_primitive::UNKNOWN;
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
      ("primitive",
       po::value<std::string>()->default_value("async_rmi"),
       "Primitive to be used (async_rmi, try_rmi)")
      ("sync",
       po::value<std::string>()->default_value("async_rmi"),
       "Primitive to be used for synchronization (async_rmi, sync_rmi, "
       "rmi_fence, ping_pong)")
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
      else if (s=="ping_pong") {
        spr = rmi_primitive::PING_PONG;
      }
      else {
        if (get_location_id()==0)
          error("%s: Incorrect sync primitive.\n", argv[0]);
        return EXIT_FAILURE;
      }
    }

    if (vm.count("use_stack"))
      use_stack = true;
  }
  catch (boost::program_options::unknown_option& ex) {
    error("%s: %s.\n", argv[0], ex.what());
  }

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
  if (pr == rmi_primitive::TRY_RMI)
    flags |= allow_try_rmi;

  distributed_object d_obj{flags, s_buf};

  if (get_location_id() == 0) {
    std::fprintf(stdout, BENCHMARK, get_primitive_name(pr));
    std::fprintf(stdout, "# Synchronization: %s\n", get_primitive_name(spr));
    std::fprintf(stdout, "# Aggregation: %s\n", (aggregation ? "yes" : "no"));
    std::fprintf(stdout, "# Intragang: %s\n", (src_diff_gang ? "no" : "yes"));
    std::fprintf(stdout, "%-*s%*s\n",
                 10, "# Size", FIELD_WIDTH, "Latency (us)");
    std::fflush(stdout);
  }

  // latency benchmark
  for (int size = 0; size <= MAX_SIZE; size = (size ? size * 2 : size + 1)) {
    if (size > LARGE_MESSAGE_SIZE) {
      loop = LOOP_LARGE;
      skip = SKIP_LARGE;
    }

    d_obj.set_counter(skip + loop);

    rmi_fence(); // quiescence before benchmarking

    switch (pr) {
      case rmi_primitive::ASYNC_RMI:
        switch (spr) {
          case rmi_primitive::ASYNC_RMI:
            async_rmi_os(skip, loop, src_diff_gang, true, d_obj, size);
            break;
          case rmi_primitive::SYNC_RMI:
            async_rmi_os(skip, loop, src_diff_gang, false, d_obj, size);
            break;
          case rmi_primitive::FENCE:
            async_rmi_fence(skip, loop, src_diff_gang, d_obj, size);
            break;
          case rmi_primitive::PING_PONG:
            async_rmi_pp(skip, loop, src_diff_gang, d_obj, size);
            break;
          default:
            error("%s: Incorrect sync primitive.\n", argv[0]);
            break;
        }
        break;
      case rmi_primitive::TRY_RMI:
        switch (spr) {
          case rmi_primitive::ASYNC_RMI:
            try_rmi_os(skip, loop, src_diff_gang, true, d_obj, size);
            break;
          case rmi_primitive::SYNC_RMI:
            try_rmi_os(skip, loop, src_diff_gang, false, d_obj, size);
            break;
          case rmi_primitive::FENCE:
            try_rmi_fence(skip, loop, src_diff_gang, d_obj, size);
            break;
          case rmi_primitive::PING_PONG:
            try_rmi_pp(skip, loop, src_diff_gang, d_obj, size);
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
