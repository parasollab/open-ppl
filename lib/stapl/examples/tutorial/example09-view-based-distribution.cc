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
#include <iomanip>
#include <stapl/array.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include "../../../benchmarks/utilities/confint.hpp"

typedef stapl::counter<stapl::default_timer> counter_t;
typedef confidence_interval_controller       controller;

// Define the type of an array of integers ready for data distribution.
typedef stapl::array<size_t,
                     stapl::view_based_partition<stapl::distribution_spec<>>,
                     stapl::view_based_mapper<stapl::distribution_spec<>>
                    >                        array_type;
typedef stapl::array_view<array_type >       array_view_type;

// A class that provides the p_object method to determine the current
// location id.
struct beacon
  : public stapl::p_object
{ };

struct assign_id
{
private:
  // Required to determine which location is working on which element.
  stapl::p_object_pointer_wrapper<beacon> m_beacon;

public:
  assign_id(beacon* b)
    : m_beacon(b)
  { }

  // Function that sets the view element referenced by "i" to the id of the
  // location where it is stored.
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

template<typename T>
void container_test(const T &distribution, controller &ctrl)
{
  // Counter used to measure execution time.
  counter_t count;

  while (ctrl.iterate())
  {
    count.reset();
    count.start();

    // Construction of a STAPL array of arr_size elements using Block
    // distribution.
    array_type array_container(distribution);

    count.stop();
    ctrl.push_back(count.value());

    // Required to synchronize locations
    stapl::rmi_fence();
  }
}

void paragraph_test(array_view_type &view, controller &ctrl)
{
  // Counter used to measure execution time.
  counter_t count;

  // Create a p_object that can be used by the work function to determine
  // the location on which a task is executing.
  beacon b;

  // stapl::map_func() applies the assign_id::operator() to each element on
  // the array_view. Note that each location will work on its distributed
  // data.
  while(ctrl.iterate())
  {
    count.reset();
    count.start();
    stapl::map_func(assign_id(&b), view);
    count.stop();
    ctrl.push_back(count.value());

    // Required to synchronize locations
    stapl::rmi_fence();
  }
}

stapl::exit_code stapl_main(int argc, char **argv)
{
  // Check for input parameters.
  if (argc < 3) {
    stapl::do_once(
      [] { std::cout << "usage: ./exec array_size block_size" << std::endl; }
    );
    return EXIT_FAILURE;
  }

  // Counter used to measure execution time.
  counter_t count;

  // Stores the time samples.
  const int samples = 32;

  controller ctrl_block(samples, samples, 0.05);
  controller ctrl_cyclic(samples, samples, 0.05);
  controller ctrl_block_cyclic(samples, samples, 0.05);
  controller ctrl_balance(samples, samples, 0.05);

  controller ctrl_block_func(samples, samples, 0.05);
  controller ctrl_cyclic_func(samples, samples, 0.05);
  controller ctrl_block_cyclic_func(samples, samples, 0.05);
  controller ctrl_balance_func(samples, samples, 0.05);

  controller testing(samples, samples, 0.05);

  // Definition of number of elements and block size for distributions.
  std::size_t arr_size = atol(argv[1]);
  std::size_t block_size = atol(argv[2]);

  stapl::do_once(
    [] {std::cout << "Starting container creation test ...\n";}
  );

  container_test(stapl::block(arr_size, block_size), ctrl_block);
  container_test(stapl::cyclic(arr_size), ctrl_cyclic);
  container_test(stapl::block_cyclic(arr_size, block_size), ctrl_block_cyclic);
  container_test(stapl::balance(arr_size), ctrl_balance);

  stapl::do_once(
    [] { std::cout << "Container test successfully completed.\n\n";
         std::cout << "Starting paragraph test ...\n";}
  );

  // Construction of STAPL arrays.
  array_type array_block(stapl::block(arr_size, block_size));
  array_type array_cyclic(stapl::cyclic(arr_size));
  array_type array_block_cyclic(stapl::block_cyclic(arr_size, block_size));
  array_type array_balance(stapl::balance(arr_size));

  // Construction of STAPL array views.
  array_view_type block_view(array_block);
  array_view_type cyclic_view(array_cyclic);
  array_view_type block_cyclic_view(array_block_cyclic);
  array_view_type balance_view(array_balance);

  // Paragraph execution tests.
  paragraph_test(block_view, ctrl_block_func);
  paragraph_test(cyclic_view, ctrl_cyclic_func);
  paragraph_test(block_cyclic_view, ctrl_block_cyclic_func);
  paragraph_test(balance_view, ctrl_balance_func);

  // Print the location of the distributed containers.
  stapl::do_once(

    [&]( ){
       std::cout << "Paragraph test successfully completed.\n\n";

       // Get container size.
       std::size_t a_sz = block_view.size();

       // Temporal variable needed for formatting output.
       std::size_t tmp = a_sz;

       if (arr_size < 51)
       {
         std::cout << "Printing locations of distributions: " << std::endl
                   << std::endl;

         // Formatting output.
         while (0 < tmp )
         {
           std::cout << " ";
           tmp /= 10;
         }

         std::cout << "       \tBlock \tCyclic \tBlock-Cyclic \tBalance"
                   << std::endl;

         // Print which location is working with each container element.
         for (unsigned int i = 0; i < a_sz; ++i)
         {
            std::cout << "Array[" << i << "]\t"<< std::setw(5) << block_view[i]
                      << "\t" << std::setw(6) << cyclic_view[i] << "\t"
                      << std::setw(12) << block_cyclic_view[i] << "\t"
                      << std::setw(7) << balance_view[i] << std::endl;
         }
         std::cout << std::endl;
       }

       std::string report_block;
       report_block += "\nTest: Container construction. Block distribution.\n";

       std::string report_cyclic;
       report_cyclic += "\nTest: Container construction.";
       report_cyclic += " Cyclic distribution.\n";

       std::string report_block_cyclic;
       report_block_cyclic += "\nTest: Container construction.";
       report_block_cyclic += " Block-Cyclic distribution.\n";

       std::string report_balance;
       report_balance += "\nTest: Container construction.";
       report_balance += " Balance distribution.\n";


       std::string report_block_func;
       report_block_func += "\nTest: Paragraph execution.";
       report_block_func += " Block distribution.\n";

       std::string report_cyclic_func;
       report_cyclic_func += "\nTest: Paragraph execution.";
       report_cyclic_func += " Cyclic distribution.\n";

       std::string report_block_cyclic_func;
       report_block_cyclic_func += "\nTest: Paragraph execution.";
       report_block_cyclic_func += " Block-Cyclic distribution.\n";

       std::string report_balance_func;
       report_balance_func += "\nTest: Paragraph execution.";
       report_balance_func += " Balance distribution.\n";

       std::cout << "Test report:" << std::endl;

       ctrl_block.report(report_block);
       ctrl_cyclic.report(report_cyclic);
       ctrl_block_cyclic.report(report_block_cyclic);
       ctrl_balance.report(report_balance);

       ctrl_block_func.report(report_block_func);
       ctrl_cyclic_func.report(report_cyclic_func);
       ctrl_block_cyclic_func.report(report_block_cyclic_func);
       ctrl_balance_func.report(report_balance_func);

       std::cout << std::endl;
    }
  );

  return EXIT_SUCCESS;
}
