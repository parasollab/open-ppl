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
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>

// A class that provides the p_object method to determine the current
// location id.
struct beacon
  : public stapl::p_object
{ };

struct assign_id
{
private:
  // Beacon is used to determine which location is working on which element.
  stapl::p_object_pointer_wrapper<beacon> m_beacon;

public:
  // Construct the work function with the specified location id.
  // The use of a data member in the work function requires the
  // definition of the define_type method below.
  assign_id(beacon* b)
    : m_beacon(b)
  { }

  // Function that modifies the view to store the id of the location that
  // contains the data. We added "T i" as an access to the data contained in
  // the view.
  template<typename T>
  void operator()(T i)
  {
    i = m_beacon->get_location_id();
  }

  // Function used to serialize work function instances as part of sending
  // them between locations.
  void define_type(stapl::typer& t)
  {
    t.member(m_beacon);
  }
};

stapl::exit_code stapl_main(int argc, char **argv)
{
  // Check for input parameters. The primary use of stapl::do_once() is to
  // allow formatted output from a single location in an SPMD section.
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

  // Define the type of an array of integers ready for data distribution.
  using array_type = stapl::array<
                       int,
                       stapl::view_based_partition<stapl::distribution_spec<>>,
                       stapl::view_based_mapper<stapl::distribution_spec<>>
                     >;

  // Construction of a STAPL array of arr_size elements distributed
  // in (arr_size/block_size) blocks cyclically between all locations.
  array_type array_ints(stapl::block_cyclic(arr_size, block_size));

  // Construction of a STAPL array view.
  // Note that we need the container and its type to declare a view.
  // We can use the view to access the information about the container
  // and its data.
  stapl::array_view<array_type > array_view(array_ints);

  // Create a p_object that can be used by the work function to determine
  // the location on which a task is executing.
  beacon b;

  // stapl::map_func() applies the function defined by operator() in
  // the struct assign_id to each element on the array_view.
  // Note that each location will work on its own data.
  stapl::map_func(assign_id(&b), array_view);

  // Print container data. The primary use of stapl::do_once() is to
  // allow formatted output from a single location in an SPMD section.
  // This function should not be used regularly in applications,
  // because all locations wait until its conclusion, i.e., no parallel
  // computation is occurring.
  stapl::do_once(

    // Use lambda function to print in stapl::do_once().
    [](stapl::array_view<array_type > array_view){

       // Get container size through its views.
       std::size_t a_sz = array_view.size();

       // Print data through its views.
       for (std::size_t i = 0; i < a_sz; ++i)
       {
          std::cout << "Array[" << i << "] belongs to Location "
                    << array_view[i] << std::endl;
       }
    }
    // stapl::do_once() parameters.
    , array_view);

  return EXIT_SUCCESS;
}
