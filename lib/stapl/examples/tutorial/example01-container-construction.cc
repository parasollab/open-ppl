/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <algorithm>
#include <utility>
#include <iostream>
#include <stapl/vector.hpp>
#include <stapl/array.hpp>
#include <stapl/utility/do_once.hpp>

stapl::exit_code stapl_main(int argc, char **argv)
{
  // Construction of one STAPL vector of strings with 5 elements and
  // one STAPL array of integers with 10 elements. Note that STAPL
  // containers are in the stapl namespace and that STL components in
  // the std namespace. STL components can be used as container elements.
  stapl::vector<std::string> vec_strings(5, "Howdy");
  stapl::array<int> array_ints(10, 7);

  // Obtain the sizes of the containers.
  std::size_t v_sz = vec_strings.size();
  std::size_t a_sz = array_ints.size();

  // Print container data. The primary use of stapl::do_once() is to
  // allow formatted output from a single location in an SPMD section.
  // This function should not be used regularly in applications,
  // because all locations wait until its conclusion, i.e., no parallel
  // computation is occurring.
  stapl::do_once(

    // Use lambda function to print in stapl::do_once().
    [](std::size_t v_sz, std::size_t a_sz){
      std::cout << "Size of Vector of Strings: " << v_sz << std::endl;
      std::cout << "Size of Array of Integers: " << a_sz << std::endl;
    }

  // stapl::do_once() parameters
  , v_sz, a_sz);

  return EXIT_SUCCESS;
}
