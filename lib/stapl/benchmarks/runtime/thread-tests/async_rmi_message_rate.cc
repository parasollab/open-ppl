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
/// Measures message rates between locations. The locations on the even numbered
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


class distributed_object
: public p_object
{
private:
  int m_count;

public:
  distributed_object(unsigned int flags)
  : p_object(flags),
    m_count(0)
  { }

  void put(void) noexcept
  { ++m_count; }

  void wait_done(void)
  {
    block_until([this] { return (this->m_count>0); });
    --m_count;
  }
};


exit_code stapl_main(int argc, char *argv[])
{
  const unsigned int nthreads = (get_num_locations() / get_num_processes());
  const unsigned int lid      = get_location_id();
  const unsigned int pid      = get_process_id();

  // default options
  const std::size_t ntimes    = 10000;
  const std::size_t wait_iter = 256;
  bool simple_output          = false;
  bool aggregation            = false;

  // parse options
  try {
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
      ("help",
       "Print this help message")
      ("aggregation",
       "Enable aggregation")
      ("simple_output",
       "Print only for location 0");

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

    if (vm.count("simple_output"))
      simple_output = true;
  }
  catch (boost::program_options::unknown_option& ex) {
    std::cerr << argv[0] << ": " << ex.what() << '\n';
  }

  distributed_object d(aggregation ? 0 : no_aggregation);
  rmi_handle::reference const& r = d.get_rmi_handle();

  if (lid==0) {
    std::printf("# Locations: %u\n", get_num_locations());
  }

  rmi_fence(); // quiescence before benchmarking

  if (pid%2==0) {
    const unsigned int dest = (lid + nthreads);
    counter<default_timer> timer;

    timer.start();
    for (std::size_t i = 0; i < ntimes; ++i) {
      async_rmi(dest, r, &distributed_object::put);
      if (i % wait_iter == 0) {
       d.wait_done();
      }
    }
    d.wait_done();
    const double time = (timer.stop() / ntimes);

    if (!simple_output)
      std::printf("Location %u message rate %f messages/sec\n",
                  lid, (1.0/time));
    else if (lid==0)
      std::printf("%f\n", (1.0/time));
  }
  else {
    const unsigned int dest = (lid - nthreads);

    for (std::size_t i = 0; i < ntimes; ++i) {
      d.wait_done();
      if (i % wait_iter == 0) {
        async_rmi(dest, r, &distributed_object::put);
      }
    }
    async_rmi(dest, r, &distributed_object::put);
  }

  rmi_fence(); // wait for all requests to finish

  return EXIT_SUCCESS;
}
