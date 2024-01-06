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
/// Measures latency between locations. The locations on the even numbered
/// processes send RMIs to the locations on the odd numbered processes and wait
/// for a reply.
///
/// Based on benchmarks from the paper "Test Suite for Evaluating Performance of
/// Multithreaded MPI Communication", Thakur, R, Gropp, WD., Parallel Computing,
/// 2009.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <cstdio>
#include <functional>
#include <iostream>
#include <vector>
#include <boost/program_options.hpp>


using namespace stapl;


//////////////////////////////////////////////////////////////////////
/// @brief Object to transport raw bytes of memory.
//////////////////////////////////////////////////////////////////////
class window
{
private:
  const char* m_buf;
  std::size_t m_size;

public:
  constexpr window(const char* buf, std::size_t size) noexcept
  : m_buf(buf),
    m_size(size)
  { }

  std::size_t size(void) const noexcept
  { return m_size; }

  void define_type(stapl::typer& t)
  {
    t.member(m_buf, m_size);
    t.member(m_size);
  }
};


class distributed_object
: public p_object
{
private:
  bool m_done;

public:
  distributed_object(void)
  : p_object(no_aggregation),
    m_done(false)
  { }

  void put(window const&) noexcept
  { m_done = true; }

  void wait_done(void)
  {
    block_until([this] { return this->m_done; });
    m_done = false;
  }
};


exit_code stapl_main(int argc, char *argv[])
{
  const unsigned int nthreads = (get_num_locations() / get_num_processes());
  const unsigned int lid      = get_location_id();
  const unsigned int pid      = get_process_id();

  // default options
  std::size_t max_size        = 1024;
  const std::size_t ntimes    = 1000;
  std::size_t incr            = 16;
  const std::size_t threshold = 256;
  const std::size_t big_incr  = 64;
  std::function<std::size_t(std::size_t, std::size_t)> inc_fun =
    [](std::size_t s, std::size_t incr) { return (s + incr); };

  // parse options
  try {
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
      ("help",
       "Print this help message")
      ("max_size",
       po::value<std::size_t>(),
       "Maximum payload size (will be implicitly change the algorithm to "
       "double the size for each iteration.");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      if (get_location_id()==0)
        std::cout << desc << std::endl;
      return EXIT_SUCCESS;
    }

    if (vm.count("max_size")) {
      max_size = vm["max_size"].as<std::size_t>();
      inc_fun = [](std::size_t s, std::size_t) { return (s==0 ? 1 : s * 2); };
    }
  }
  catch (boost::program_options::unknown_option& ex) {
    std::cerr << argv[0] << ": " << ex.what() << '\n';
  }

  const std::vector<char> v(max_size);

  distributed_object d;
  rmi_handle::reference const& r = d.get_rmi_handle();

  if (lid==0) {
    std::printf("# Locations: %u\n", get_num_locations());
    std::printf("# Size (bytes) \t Time (us)\n");
  }

  for (std::size_t size=0; size<=max_size; size=inc_fun(size, incr)) {
    const window w(&v[0], size);
    counter<default_timer> timer;

    rmi_fence(); // quiescence before benchmarking

    if (pid%2==0) {
      const unsigned int dest = (lid + nthreads);
      timer.start();
      for (std::size_t i=0; i<ntimes; ++i) {
        async_rmi(dest, r, &distributed_object::put, w);
        d.wait_done();
      }
      const double time = timer.stop();

      if (lid==0) {
        const double ttime = (time / (2.0*ntimes));
        std::printf("%d \t %f\n", int(size), (ttime * 1000000.0));
      }
    }
    else {
      const unsigned int dest = (lid - nthreads);
      for (std::size_t i=0; i<ntimes; ++i) {
        d.wait_done();
        async_rmi(dest, r, &distributed_object::put, w);
      }
    }

    if (size==threshold)
      incr = big_incr;
  }

  rmi_fence(); // wait for all requests to finish

  return EXIT_SUCCESS;
}
