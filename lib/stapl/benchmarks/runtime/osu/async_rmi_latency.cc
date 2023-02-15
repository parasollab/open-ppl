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
/// Benchmark for @ref stapl::async_rmi(), @ref stapl::try_rmi(), and
/// @ref stapl::bind_rmi() latency.
///
/// The main idea of the benchmark is to send a number of RMIs with a specific
/// payload size and measure the time that it took for them to be processed.
///
/// The ping process can be compared against
/// -# @c mpi/one-sided/osu_put_latency.c
/// -# @c openshmem/osu_oshm_put.c
/// in OSU benchmarks.
///
/// The ping-pong process can be compared against
/// -# @c mpi/one-sided/osu_put_latency.c with @c MPI_Put() with
///       Post/Start/Complete/Wait or Fence
/// -# @c mpi/pt2pt/osu_latency.c
/// in OSU benchmarks.
///
/// It can also be compared against
/// -# @c upc/osu_upc_memput.c
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

//////////////////////////////////////////////////////////////////////
/// @brief Functor to invoke unordered::async_rmi
//////////////////////////////////////////////////////////////////////
struct invoke_unordered_async_rmi
{
  template<typename Ref, typename PMF, typename... Args>
  void operator()(unsigned int l, Ref&& ref, PMF&& pmf, Args&&... args) const
  {
    unordered::async_rmi(l,
                         std::forward<Ref>(ref),
                         std::forward<PMF>(pmf),
                         std::forward<Args>(args)...);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor to invoke async_rmi
//////////////////////////////////////////////////////////////////////
struct invoke_async_rmi
{
  template<typename Ref, typename PMF, typename... Args>
  void operator()(unsigned int l, Ref&& ref, PMF&& pmf, Args&&... args) const
  {
    async_rmi(l,
              std::forward<Ref>(ref),
              std::forward<PMF>(pmf),
              std::forward<Args>(args)...);
  }
};

// Prints the achieved latency
void print(int size, int loop, double t, bool ping_pong)
{
  const double latency = (ping_pong ? (t * 1.0e6 / (2.0 * loop))
                                    : (t * 1.0e6 / loop));
  std::fprintf(stdout, "%-*d%*.*f\n",
               10, size, FIELD_WIDTH, FLOAT_PRECISION, latency);
  std::fflush(stdout);
}


// Base async_rmi() with one-sided synchronization (async_rmi() or sync_rmi())
template<typename Sender>
void base_async_rmi_os(Sender&& sender, const int skip, const int loop,
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

      sender(1, r, &distributed_object::put_ack<decltype(w)>, w);
    }
    d_obj.wait_done();
  }
  else {
    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      sender(1, r, &distributed_object::put<decltype(w)>, w);
    }
    sync_rmi(1, r, &distributed_object::recv_sync);
  }
  const double t = counter.stop();

  // finish all traffic from newly created gang
  if (g)
    rmi_fence();

  print(size, loop, t, false);
}


// Base async_rmi() with rmi_fence() synchronization
template<typename Sender>
void base_async_rmi_fence(Sender&& sender, const int skip, const int loop,
                     const bool src_diff_gang,
                     distributed_object& d_obj, const std::size_t size)
{
  if (get_location_id() % 2 == 0) {
    rmi_handle::reference const& r = d_obj.get_rmi_handle();
    char* const s_buf              = d_obj.get_data_ptr();
    const auto w                   = make_range_n(s_buf, size);

    const unsigned int peer_lid = ((get_location_id()+1) % get_num_locations());

    counter<default_timer> counter;

    // if requested, generate traffic from its own gang
    boost::optional<gang> g;
    if (src_diff_gang)
      g = boost::in_place();

    touch_data(s_buf, size);

    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      sender(peer_lid, r, &distributed_object::put<decltype(w)>, w);
    }
    rmi_fence(); // wait for async_rmi calls to finish
    const double t = counter.stop();

    if (get_location_id()==0)
      print(size, loop, t, false);
  }
  else if (!src_diff_gang) {
    rmi_fence(); // wait for async_rmi calls to finish
  }
}


// Base async_rmi() ping-pong
template<typename Sender>
void base_async_rmi_pp(Sender&& sender, const int skip, const int loop,
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

    sender(1, r, &distributed_object::put_ping<decltype(w)>, w);
    d_obj.wait_done();
  }
  const double t = counter.stop();

  // finish all traffic from newly created gang
  if (g)
    rmi_fence();

  print(size, loop, t, true);
}


// async_rmi() with one-sided synchronization (async_rmi() or sync_rmi())
void async_rmi_os(const int skip, const int loop,
                  const bool src_diff_gang, const bool deferred_sync,
                  distributed_object& d_obj, const std::size_t size)
{
  base_async_rmi_os(
    invoke_async_rmi{}, skip, loop, src_diff_gang, deferred_sync, d_obj, size);
}

// unordered::async_rmi() with one-sided synchronization
// (async_rmi() or sync_rmi())
void unordered_async_rmi_os(const int skip, const int loop,
                  const bool src_diff_gang, const bool deferred_sync,
                  distributed_object& d_obj, const std::size_t size)
{
  base_async_rmi_os(invoke_unordered_async_rmi{}, skip, loop, src_diff_gang,
    deferred_sync, d_obj, size);
}

// async_rmi() with rmi_fence() synchronization
void async_rmi_fence(const int skip, const int loop,
                     const bool src_diff_gang,
                     distributed_object& d_obj, const std::size_t size)
{
  base_async_rmi_fence(
    invoke_async_rmi{}, skip, loop, src_diff_gang, d_obj, size);
}

// unordered::async_rmi() with rmi_fence() synchronization
void unordered_async_rmi_fence(const int skip, const int loop,
                     const bool src_diff_gang,
                     distributed_object& d_obj, const std::size_t size)
{
  base_async_rmi_fence(
    invoke_unordered_async_rmi{}, skip, loop, src_diff_gang, d_obj, size);
}

// async_rmi() ping-pong
void async_rmi_pp(const int skip, const int loop,
                  const bool src_diff_gang,
                  distributed_object& d_obj, const std::size_t size)
{
  base_async_rmi_pp(invoke_async_rmi{}, skip, loop, src_diff_gang, d_obj, size);
}

// unordered::async_rmi() ping-pong
void unordered_async_rmi_pp(const int skip, const int loop,
                  const bool src_diff_gang,
                  distributed_object& d_obj, const std::size_t size)
{
  base_async_rmi_pp(
    invoke_unordered_async_rmi{}, skip, loop, src_diff_gang, d_obj, size);
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

      try_rmi(1, r, &distributed_object::put_ack<decltype(w)>, w);
    }
    d_obj.wait_done();
  }
  else {
    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      try_rmi(1, r, &distributed_object::put<decltype(w)>, w);
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
  if (get_location_id() % 2 == 0) {
    rmi_handle::reference const& r = d_obj.get_rmi_handle();
    char* const s_buf              = d_obj.get_data_ptr();
    const auto w                   = make_range_n(s_buf, size);

    counter<default_timer> counter;

    const unsigned int peer_lid = ((get_location_id()+1) % get_num_locations());

    // if requested, generate traffic from its own gang
    boost::optional<gang> g;
    if (src_diff_gang)
      g = boost::in_place();

    touch_data(s_buf, size);

    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      try_rmi(peer_lid, r, &distributed_object::put<decltype(w)>, w);
    }
    rmi_fence(); // wait for async_rmi calls to finish
    const double t = counter.stop();

    if (get_location_id()==0)
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

    try_rmi(1, r, &distributed_object::put_ping<decltype(w)>, w);
    d_obj.wait_done();
  }
  const double t = counter.stop();

  // finish all traffic from newly created gang
  if (g)
    rmi_fence();

  print(size, loop, t, true);
}


// bind_rmi() with one-sided synchronization (async_rmi() or sync_rmi())
void bind_rmi_os(const int skip, const int loop,
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
    auto tunnel = bind_rmi(&distributed_object::put_ack<decltype(w)>, r);
    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      tunnel(1, w);
    }
    tunnel.flush();
    d_obj.wait_done();
  }
  else {
    auto tunnel = bind_rmi(&distributed_object::put<decltype(w)>, r);
    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      tunnel(1, w);
    }
    tunnel.flush();
    sync_rmi(1, r, &distributed_object::recv_sync);
  }
  const double t = counter.stop();

  // finish all traffic from newly created gang
  if (g)
    rmi_fence();

  print(size, loop, t, false);
}


// bind_rmi() with rmi_fence() synchronization
void bind_rmi_fence(const int skip, const int loop,
                    const bool src_diff_gang,
                    distributed_object& d_obj, const std::size_t size)
{
  if (get_location_id() % 2 == 0) {
    rmi_handle::reference const& r = d_obj.get_rmi_handle();
    char* const s_buf              = d_obj.get_data_ptr();
    const auto w                   = make_range_n(s_buf, size);

    counter<default_timer> counter;

    const unsigned int peer_lid = ((get_location_id()+1) % get_num_locations());

    // if requested, generate traffic from its own gang
    boost::optional<gang> g;
    if (src_diff_gang)
      g = boost::in_place();

    auto tunnel = bind_rmi(&distributed_object::put<decltype(w)>, r);

    touch_data(s_buf, size);

    for (int i = 0; i < skip + loop; i++) {
      if (i == skip)
        counter.start();

      tunnel(peer_lid, w);
    }
    tunnel.flush();
    rmi_fence(); // wait for async_rmi calls to finish
    const double t = counter.stop();

    if (get_location_id()==0)
      print(size, loop, t, false);
  }
  else if (!src_diff_gang) {
    rmi_fence(); // wait for async_rmi calls to finish
  }
}


// bind_rmi() ping-pong
void bind_rmi_pp(const int skip, const int loop,
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

  auto tunnel = bind_rmi(&distributed_object::put_ping<decltype(w)>, r);

  for (int i = 0; i < skip + loop; i++) {
    if (i == skip)
      counter.start();

    tunnel(1, w);
    tunnel.flush();
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
  int skip = 1000;
  int loop = 10000;

  // options
  bool aggregation   = false;
  bool src_diff_gang = false;
  rmi_primitive pr   = rmi_primitive::UNKNOWN;
  rmi_primitive spr  = rmi_primitive::UNKNOWN;
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
       po::value<std::string>()->default_value("async_rmi"),
       "Primitive to be used (async_rmi, try_rmi, bind_rmi, unordered)")
      ("sync",
       po::value<std::string>()->default_value("async_rmi"),
       "Primitive to be used for synchronization (async_rmi, sync_rmi, "
       "rmi_fence, ping_pong)")
      ("use_stack",
       "If defined, then the buffer is created on the stack.")
      ("use_pairs",
       "If defined, half of the locations are sending elements to the other "
       "half. \"--sync rmi_fence\" is implied");

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
      else if (s=="bind_rmi") {
        pr = rmi_primitive::BIND_RMI;
      }
      else if (s=="unordered") {
        pr = rmi_primitive::UNORDERED_ASYNC_RMI;
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

    if (vm.count("use_pairs"))
      use_pairs = true;
  }
  catch (boost::program_options::unknown_option& ex) {
    error("%s: %s.\n", argv[0], ex.what());
  }

  if (!use_pairs) {
    if (get_num_locations() != 2) {
      if (get_location_id()==0)
        error("%s: This benchmark requires two locations.\n", argv[0]);
      return EXIT_FAILURE;
    }
  }
  else {
    if (get_num_locations() % 2 != 0) {
      if (get_location_id()==0)
        error("%s: This benchmark requires even number of locations.\n",
              argv[0]);
      return EXIT_FAILURE;
    }
  }

  // using pairs implies rmi_fence() based synchronization
  if (use_pairs)
    spr = rmi_primitive::FENCE;

  if (use_pairs && src_diff_gang) {
    if (get_location_id() == 0) {
      error("%s: --use_pairs is not allowed when src_diff_gang is defined.\n",
            argv[0]);
      return EXIT_FAILURE;
    }
  }

  char stack_buf[BUFFER_SIZE];
  const int page_size = getpagesize();
  assert(page_size <= MAX_ALIGNMENT);
  char* buf   = nullptr;
  char* s_buf = nullptr;
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
    if (use_pairs)
      std::fprintf(stdout, "# Pairs: %d\n", (get_num_locations() / 2));
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
      case rmi_primitive::BIND_RMI:
        switch (spr) {
          case rmi_primitive::ASYNC_RMI:
            bind_rmi_os(skip, loop, src_diff_gang, true, d_obj, size);
            break;
          case rmi_primitive::SYNC_RMI:
            bind_rmi_os(skip, loop, src_diff_gang, false, d_obj, size);
            break;
          case rmi_primitive::FENCE:
            bind_rmi_fence(skip, loop, src_diff_gang, d_obj, size);
            break;
          case rmi_primitive::PING_PONG:
            bind_rmi_pp(skip, loop, src_diff_gang, d_obj, size);
            break;
          default:
            error("%s: Incorrect sync primitive.\n", argv[0]);
            break;
        }
        break;

      case rmi_primitive::UNORDERED_ASYNC_RMI:
        switch (spr) {
          case rmi_primitive::ASYNC_RMI:
            unordered_async_rmi_os(skip, loop, src_diff_gang, true, d_obj,
                                   size);
            break;
          case rmi_primitive::SYNC_RMI:
            unordered_async_rmi_os(skip, loop, src_diff_gang, false, d_obj,
                                   size);
            break;
          case rmi_primitive::FENCE:
            unordered_async_rmi_fence(skip, loop, src_diff_gang, d_obj, size);
            break;
          case rmi_primitive::PING_PONG:
            unordered_async_rmi_pp(skip, loop, src_diff_gang, d_obj, size);
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

    rmi_fence(); // wait for RMI calls to finish
  }

  delete[] buf;

  return EXIT_SUCCESS;
}
