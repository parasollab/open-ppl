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
  // one STAPL array of integers with 10 elements.
  stapl::vector<std::string> vec_strings(5, "Howdy");
  stapl::array<int> array_ints(10, 7);

  // Construction of one STAPL vector view and one STAPL array view.
  // Note that we need the container and its type to declare a view.
  // Once in a view, we can access the information of the container
  // through the view.
  stapl::vector_view<stapl::vector<std::string> > vector_view(vec_strings);
  stapl::array_view<stapl::array<int> > array_view(array_ints);

  // Print container data. The primary use of stapl::do_once() is to
  // allow formatted output from a single location in an SPMD section.
  // This function should not be used regularly in applications,
  // because all locations wait until its conclusion, i.e., no parallel
  // computation is occurring.
  stapl::do_once(

    // Use lambda function to print in stapl::do_once().
    [](stapl::vector_view<stapl::vector<std::string> > vector_view,
       stapl::array_view<stapl::array<int> > array_view){

       // Pop back one "Howdy" and push back "Gig 'em" in the vector view.
       vector_view.pop_back();
       vector_view.push_back("Gig 'em");

       // Change the first 7 for one 11 in the array view.
       array_view[0] = 11;

       // Get container sizes through their views.
       std::size_t v_sz = vector_view.size();
       std::size_t a_sz = array_view.size();

       // Print data through their views.
       for (std::size_t i = 0; i < v_sz; ++i)
       {
          std::cout << "Vector[" << i << "] = " << vector_view[i] << std::endl;
       }

       for (std::size_t i = 0; i < a_sz; ++i)
       {
          std::cout << "Array[" << i << "] = " << array_view[i] << std::endl;
       }

    }

  // stapl::do_once() parameters
  , vector_view, array_view);

  return EXIT_SUCCESS;
}
