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
/// Benchmark to evaluate the memory footprint of the STAPL RTS.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (stapl::get_location_id()==0) {
    std::cout << "kernel    " << "STAPL" << std::endl;
    std::cout << "elements  " << stapl::get_num_locations() << std::endl;
    std::cout << "processes " << stapl::get_num_processes() << '\n'
              << "threads   "
              << (stapl::get_num_locations()/ stapl::get_num_processes())
              << std::endl;
  }

  return EXIT_SUCCESS;
}
