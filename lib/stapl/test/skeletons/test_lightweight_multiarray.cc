/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>

#include <stapl/runtime.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/skeletons/utility/lightweight_multiarray.hpp>

#include "../test_report.hpp"

using namespace stapl;

using timer_type = counter<default_timer>;


exit_code stapl_main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cout<< "usage: <exec> <n>" <<std::endl;
    exit(1);
  }

  std::size_t n = atol(argv[1]);

  auto dims = make_tuple(n, n);

  //
  // Multiarray construction with initialization
  //
  timer_type init_timer;
  init_timer.start();

  lightweight_multiarray<double, 2> init_ma(dims);

  const double init_elapsed = init_timer.stop();


  //
  // Multiarray fill with initialization
  //
  timer_type init_fill_timer;
  init_fill_timer.start();

  for (size_t i=0; i<n; i++)
    for (size_t j=0; j<n; j++)
      init_ma(i,j) = 1;

  const double init_fill_elapsed = init_fill_timer.stop();


  //
  // Multiarray construction without initialization
  //
  timer_type no_init_timer;
  no_init_timer.start();

  lightweight_multiarray<double, 2, non_initialized> no_init_ma(dims);

  const double no_init_elapsed = no_init_timer.stop();

  //
  // Multiarray fill without initialization
  //
  timer_type no_init_fill_timer;
  no_init_fill_timer.start();

  for (size_t i=0; i<n; i++)
    for (size_t j=0; j<n; j++)
      no_init_ma(i,j) = 1;

  const double no_init_fill_elapsed = no_init_fill_timer.stop();

  do_once([&]()
  {
    std::cout << "=========================================\n";
    std::cout << "lightweight_multiarray construction times\n";
    std::cout << "=========================================\n";
    std::cout << "  initialized storage     = " << init_elapsed << "\n";
    std::cout << "  non-initialized storage = " << no_init_elapsed << "\n";
    std::cout << "\n=========================================\n";
    std::cout << "lightweight_multiarray fill times\n";
    std::cout << "=========================================\n";
    std::cout << "  initialized storage     = " << init_fill_elapsed << "\n";
    std::cout << "  non-initialized storage = " << no_init_fill_elapsed << "\n";
  });

  return EXIT_SUCCESS;
}
