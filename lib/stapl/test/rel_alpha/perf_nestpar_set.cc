/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cstdio>
#include <cstdlib>

#include <iostream>

#include <stapl/utility/tuple.hpp>
#include <stapl/domains/indexed.hpp>

#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>

#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>

#include <stapl/views/map_view.hpp>
#include <stapl/containers/map/map.hpp>

#include <stapl/containers/set/set.hpp>

#include <stapl/skeletons/serial.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include <stapl/runtime.hpp>

#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include "json_parser.hpp"

#include <test/algorithms/test_report.h>

#include "perf_nestpar/test_fixture.hpp"

// Test Group 1 : Insert with a counting_view/map_func the elements
#define PERF_TEST_01
#include "perf_nestpar/test_1.hpp"
// Test Group 2 :
#define PERF_TEST_02
#include "perf_nestpar/test_2.hpp"
#define PERF_TEST_03
#include "perf_nestpar/test_3.hpp"
#define PERF_TEST_04
#include "perf_nestpar/test_4.hpp"
#define PERF_TEST_05
#include "perf_nestpar/test_5.hpp"
// Test Group 3 : 2lvl - Insert with a counting_view/map_func the elements
#define PERF_TEST_06
#include "perf_nestpar/test_6.hpp"


using namespace std;

char *opt_data = 0;

stapl::exit_code stapl_main(int argc, char **argv)
{

  for ( int argi = 1; argi < argc; ) {
    char * opt = argv[argi++];
    if ('-' == opt[0] ) {
      switch ( opt[1] ) {
      case 'h':
        cerr << "Use -data tiny/small/medium/big/huge \n";
        break;
      case 'd':
        opt_data = argv[argi++];
        break;
      }
    } else {
      cerr << "unknown command line argument " << opt << endl;
    }
  }

  int model = -1;
  switch ( opt_data[0] ) {
  case 't':
    model = 1;
    break;
  case 's':
    model = 100;
    break;
  case 'm':
    model = 10000;
    break;
  case 'b':
    model = 10000000;
    break;
  case 'h':
    model = 100000000;
    break;
  default:
    cerr << "opt_data " << opt_data << endl;
    break;
  }
  if (model == -1) {
    std::cerr << "usage: exe -data tiny/small/medium/big/huge\n";
    exit(1);
  }

  int first_test = 1;
  int last_test = 6;

  for (int test = first_test; test <= last_test; ++test) {

    std::stringstream test_ss ;
    test_ss << "\nTest: #" << test << std::endl;
    test_report(test_ss.str());
    stapl::rmi_fence();
    switch ( test) {
    case 1:
      #ifdef PERF_TEST_01
      perf_test_01(model);
      #endif
      break;
    case 2:
      #ifdef PERF_TEST_02
      perf_test_02(model);
      #endif
      break;
    case 3:
      #ifdef PERF_TEST_03
      perf_test_03(model);
      #endif
      break;
    case 4:
      #ifdef PERF_TEST_04
      perf_test_04(model);
      #endif
      break;
    case 5:
      #ifdef PERF_TEST_05
      perf_test_05(model);
      #endif
      break;
    case 6:
      #ifdef PERF_TEST_06
      perf_test_06(model);
      #endif
      break;
    default:
      std::stringstream not_impl ;
      not_impl << "ERROR: - test " << test << " not yet implemented - \n";
      test_report(not_impl.str());
      break;

    }
    stapl::rmi_fence();
  }


  return EXIT_SUCCESS;
}
