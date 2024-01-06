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
#include <cmath>
#include <stapl/vector.hpp>
#include <stapl/array.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/runtime.hpp>

stapl::exit_code stapl_main(int argc, char **argv)
{
  // Check for input parameters.
  if (argc < 3) {
    stapl::do_once(
      [] { std::cout << "usage: ./exec array_size block_size" << std::endl; }
    );
    return EXIT_FAILURE;
  }

  // Definition of number of elements and block size for
  // block distribution.
  long arr_size = atol(argv[1]);
  long block_size = atol(argv[2]);

  // Integers that will store the result of future assignments.
  int result_arr = 0;
  int result_vec = 0;

  // Obtain the number of locations and the id of individual location.
  long num_loc = stapl::get_num_locations();
  long loc_id = stapl::get_location_id();

  // Define the type of an array of integers ready for data distribution.
  using array_type = stapl::array<
                       int,
                       stapl::view_based_partition<stapl::distribution_spec<>>,
                       stapl::view_based_mapper<stapl::distribution_spec<>>
                     >;

  // Construction of a STAPL array of arr_size elements distributed
  // in (arr_size/block_size) blocks between all locations and a STAPL
  // vector of integers using the default balanced distribution. These
  // are examples of collective container method use.
  array_type array_ints(stapl::block(arr_size, block_size));
  stapl::vector<int> vector_ints(arr_size);

  // Constructor of a STD vector of "stapl::future". "stapl::future"
  // allows to access the result of asynchronous operations.
  std::vector< stapl::future<int> > future_ints(arr_size);
  stapl::future<int> f_arr;
  stapl::future<int> f_vec;

  // Example of asynchronous concurrent method through "set_element".
  // This example stores even numbers at even array indices in increasing
  // order and odd numbers at odd vector indices backwards, leaving zeros
  // at odd array indices and even vector indices.
  for (int i = 0; i < arr_size; ++i)
  {
    if ( i % num_loc == loc_id)
    {
       if ( i % 2 == 0)
         array_ints.set_element(i, i);
       else
         vector_ints.set_element(i,
                    arr_size % 2 ? arr_size - i - 1 : arr_size - i);
    }
  }

  // Example of split-phase method use. "get_element_split(i)" returns
  // a stapl::future that holds the value located at index "i".
  for (int i = 0; i < arr_size; ++i)
  {
    if ( i % num_loc == loc_id)
    {
       if ( i % 2 == 0)
         future_ints[i] = array_ints.get_element_split(i);
       else
         future_ints[i] = vector_ints.get_element_split(i);
    }
  }

  // Collect the results of split-phase operations. The function "wait()"
  // does not return until the return value from the operation is received,
  // and function "get()" retrieves the value held by stapl::future.
  for (int i = 0; i < arr_size; ++i)
  {
    if ( i % num_loc == loc_id)
    {
       if ( i % 2 == 0)
       {
         future_ints[i].wait();
         vector_ints[i] = future_ints[i].get();
       }
       else
       {
         array_ints[i] = future_ints[i].get();
       }
    }
  }

  // Example of split-phase method use. "get_element_split(i)" returns
  // a stapl::future that holds the value located at index "i".
  f_arr = array_ints.get_element_split(0);
  f_vec = vector_ints.get_element_split(0);

  // "async_then" allows one to execute the function once the result of the
  // stapl::future is available.
  f_arr.async_then([&](stapl::future<int> f){
            result_arr = f.get();
            for ( int i = 1; i < arr_size; ++i)
            {
              // Example of synchronous concurrent method use.
              result_arr += array_ints.get_element(i);
            }
          });

  f_vec.async_then([&](stapl::future<int> f){
            result_vec = f.get();
            for ( int i = 1; i < arr_size; ++i)
            {
              // Example of synchronous concurrent method use.
              result_vec += vector_ints.get_element(i);
            }
          });

  // Print container and future data.
  stapl::do_once(

    // Use lambda function to print in stapl::do_once().
    [&]{

       // Get container size through its view.
       std::size_t a_sz = array_ints.size();

       // Get container size.
       std::size_t v_sz = vector_ints.size();

       // Print "array_ints" data.
       for (std::size_t i = 0; i < a_sz; ++i)
       {
         std::cout << "Array[" << i << "] = "
                   << array_ints.get_element(i) << std::endl;
       }

       // Print "vector_ints" data.
       for (std::size_t i = 0; i < v_sz; ++i)
       {
         std::cout << "Vector[" << i << "] = "
                   << vector_ints.get_element(i) << std::endl;
       }

       // Print "result_arr" and "result_vec" values. Note that
       // the results will vary due to the omission of "wait()"
       // in the assignments of "array_ints".
       std::cout << "Result array = " << result_arr << std::endl;
       std::cout << "Result vector = " << result_vec << std::endl;
     }
   );

  return EXIT_SUCCESS;
}
