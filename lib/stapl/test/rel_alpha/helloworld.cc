/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime.hpp>
#include <iostream>
#include <sstream>
#include <thread>

stapl::exit_code stapl_main(int, char*[])
{
  std::ostringstream os;

  // High-level information
  os << "Hello world from location " << stapl::get_location_id() << ' '
     << "out of " << stapl::get_num_locations() << " locations "
     << "with affinity " << stapl::get_affinity();

  // Low-level information
  os << " (process id: " << stapl::runtime::runqueue::get_process_id()
     << ", thread id: " << std::this_thread::get_id()
     << ")\n";

  std::cout << os.str();

  return EXIT_SUCCESS;
}
