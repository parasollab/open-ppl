/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/algorithms/numeric.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <iostream>
#include <sys/time.h>
#include <ctime>

#define ITERATIONS 100
#define TESTNUM 1
#define VERBOSE_VECTOR_TEST true

typedef stapl::vector<int> vec_int_tp;

using namespace std;
bool location_tests();
bool location_test(int pushbacks, int start_size);
bool many_instantiates_of_one_no_pushback();
bool many_instantiates_of_zero_no_pushback();
bool many_instantiates_of_no_size_no_pushback();
bool many_instantiates_of_one_with_one_pushback();
bool many_instantiates_of_zero_with_one_pushback();
bool many_pushbacks_onto_size_zero();
bool many_pushbacks_onto_size_one();

stapl::exit_code stapl_main(int argc, char* argv[])
{
  bool success = true;
  success = success && location_tests();
  success = success && many_instantiates_of_one_no_pushback();
  success = success && many_instantiates_of_zero_no_pushback();
  success = success && many_instantiates_of_no_size_no_pushback();
  success = success && many_instantiates_of_one_with_one_pushback();
  success = success && many_instantiates_of_zero_with_one_pushback();
  success = success && many_pushbacks_onto_size_zero();
  success = success && many_pushbacks_onto_size_one();

  if (success)
    return EXIT_SUCCESS;
  else
    return EXIT_FAILURE;
}
bool location_tests()
{
  if (location_test(1,0) && location_test(1,1) && location_test(1,2)
    && location_test(1,1000) && location_test(1000,0) && location_test(1000,1)
    && location_test(1000,2) && location_test(1000,1000))
  {
    if (stapl::get_location_id() == 0 && VERBOSE_VECTOR_TEST)
      cout << "PASS: Vector tests." << endl;
    return true;
  }
  else
  {
    if (stapl::get_location_id() == 0 && VERBOSE_VECTOR_TEST)
      cout << "--> FAIL: Vector tests." << endl;
    return false;
  }
}
bool location_test(int pushbacks, int start_size)
{
  size_t num_locs = stapl::get_num_locations();
  size_t loc = stapl::get_location_id();
  int counts[num_locs];
  bool failed = false;
  int n=pushbacks;

  vec_int_tp v(start_size);

  for (int t= loc; t < n; t+=num_locs )
  {
    v.push_back(loc);
  }

  stapl::rmi_fence();


  // Validate
  for (unsigned int t=0; t<num_locs; t++)
    counts[t]=0;
  for (int t=start_size; t< n; t++)
    counts[v[t]]++;
  for (unsigned int t=1; t<num_locs; t++)
  {
    if (counts[0] != counts[t] && counts[0] != (counts[t]+1) &&
      counts[0] != (counts[t]-1) && counts[0] != (counts[t]+2) &&
      counts[0] != (counts[t]-2))
    {
      failed = true;
      cout << counts[0] << " != " << counts[t] << " on " << t << endl;
    }
  }
  if (failed && VERBOSE_VECTOR_TEST)
    cout << "Location test(" << pushbacks << "," << start_size <<
      ") failed! Not all assignments made." << endl;

  stapl::rmi_fence();

  return !failed;
}
bool many_instantiates_of_one_no_pushback()
{
  size_t loc = stapl::get_location_id();
  stapl::counter<stapl::default_timer> execution_timer;

  for (int k=0; k < ITERATIONS; k++)
  {
    execution_timer.start();
    vec_int_tp v(1);
    stapl::rmi_fence();
    execution_timer.stop();
  }

  // Output
  if (loc == 0 && VERBOSE_VECTOR_TEST)
  {
    cout << "PASS: many_instantiates_of_one_no_pushback" << endl;
    cout << "  execution_timer.value() (" << ITERATIONS << "): "
      << execution_timer.value() << " seconds." << endl;
  }
  stapl::rmi_fence();
  return true; // Either works or crashes.
}
bool many_instantiates_of_zero_no_pushback()
{
  size_t loc = stapl::get_location_id();
  stapl::counter<stapl::default_timer> execution_timer;

  for (int k=0; k < ITERATIONS; k++)
  {
    execution_timer.start();
    vec_int_tp v(0);
    stapl::rmi_fence();
    execution_timer.stop();
  }

  // Output
  if (loc == 0 && VERBOSE_VECTOR_TEST)
  {
    cout << "PASS: many_instantiates_of_zero_no_pushback" << endl;
    cout << "  execution_timer.value() (" << ITERATIONS << "): "
      << execution_timer.value() << " seconds." << endl;
  }
  stapl::rmi_fence();
  return true; // Either works or crashes.
}
bool many_instantiates_of_no_size_no_pushback()
{
  size_t loc = stapl::get_location_id();
  stapl::counter<stapl::default_timer> execution_timer;

  for (int k=0; k < ITERATIONS; k++)
  {
    execution_timer.start();
    vec_int_tp v;
    stapl::rmi_fence();
    execution_timer.stop();
  }

  // Output
  if (loc == 0 && VERBOSE_VECTOR_TEST)
  {
    cout << "PASS: many_instantiates_of_no_size_no_pushback" << endl;
    cout << "  execution_timer.value() (" << ITERATIONS << "): "
      << execution_timer.value() << " seconds." << endl;
  }
  stapl::rmi_fence();
  return true; // Either works or crashes.
}
bool many_instantiates_of_one_with_one_pushback()
{
  size_t num_locs = stapl::get_num_locations();
  size_t loc = stapl::get_location_id();
  stapl::counter<stapl::default_timer> execution_timer;
  bool success = true;

  for (int k=0; k < ITERATIONS; k++)
  {
    execution_timer.start();
    vec_int_tp v(1);
    v.push_back(81372); // Arbitrary number
    execution_timer.stop();
    stapl::rmi_fence();

    stapl::vector_view<vec_int_tp> vw(v);
    success = num_locs ==
      stapl::count(vw, 81372) ;
  }
  stapl::rmi_fence();

  // Output
  if (loc == 0 && VERBOSE_VECTOR_TEST && success)
  {
    cout << "PASS: many_instantiates_of_one_with_one_pushback" << endl;
    cout << "  execution_timer.value() (" << ITERATIONS << "): "
      << execution_timer.value() << " seconds." << endl << flush;
  }
  else if (loc == 0 && VERBOSE_VECTOR_TEST && !success)
  {
    cout << "-->FAIL: many_instantiates_of_one_with_one_pushback" << endl;
    cout << "  execution_timer.value() (" << ITERATIONS << "): "
      << execution_timer.value() << " seconds." << endl << flush;

  }
  stapl::rmi_fence();
  return success;
}
bool many_instantiates_of_zero_with_one_pushback()
{
  size_t num_locs = stapl::get_num_locations();
  size_t loc = stapl::get_location_id();
  stapl::counter<stapl::default_timer> execution_timer;
  bool success = true;

  for (int k=0; k < ITERATIONS; k++)
  {
    execution_timer.start();
    vec_int_tp v;
    v.push_back(81372); // Arbitrary number
    stapl::rmi_fence();
    execution_timer.stop();
    size_t vsize = v.size();
    stapl::vector_view<vec_int_tp> vw(v);
    size_t count = stapl::count(vw, 81372);

    success = num_locs == count && vsize == num_locs;
    stapl::rmi_fence();
  }

  // Output
  if (loc == 0 && VERBOSE_VECTOR_TEST && success)
  {
    cout << "PASS: many_instantiates_of_zero_with_one_pushback" << endl;
    cout << "  execution_timer.value() (" << ITERATIONS << "): "
      << execution_timer.value() << " seconds." << endl;
  }
  else if (loc == 0 && VERBOSE_VECTOR_TEST && !success)
  {
    cout << "-->FAIL: many_instantiates_of_zero_with_one_pushback" << endl;
    cout << "  execution_timer.value() (" << ITERATIONS << "): "
      << execution_timer.value() << " seconds." << endl;

  }
  stapl::rmi_fence();
  return success;
}
bool many_pushbacks_onto_size_zero()
{
  int vTotal, realTotal;
  size_t num_locs = stapl::get_num_locations();
  size_t loc = stapl::get_location_id();
  bool success = true;
  stapl::counter<stapl::default_timer> execution_timer;

  for (int n=1; n < 1000; n *= 10)
  {
    execution_timer.reset();
    for (int k=0; k < ITERATIONS; k++)
    {
      // TEST: Pushback n elements.
      vTotal = 0;
      realTotal = 0;
      vec_int_tp v(0);
      execution_timer.start();
      stapl::rmi_fence();
      for (int t= loc; t < n; t+=num_locs )
      {
       v.push_back(t);
      }
      stapl::rmi_fence();
      execution_timer.stop();

      // Validate Test
      for (int t= 0; t < n; t++ )
      {
      vTotal += v[t]; // We start with a size-one vector.
      realTotal += t;
      }
      stapl::rmi_fence();

      if (vTotal != realTotal)
      success = false;
    }
    // Output
    if (loc == 0  && VERBOSE_VECTOR_TEST)
    {
      if (success)
      cout << "PASS: many_pushbacks_onto_size_zero: n=" << n << endl;
      else
      cout << "-->FAIL: many_pushbacks_onto_size_zero: n=" << n
        << ": " <<  vTotal << " != " << realTotal << endl;

      cout << "  execution_timer.value() (" << ITERATIONS << "): "
        << execution_timer.value() << " seconds." << endl;
    }
    stapl::rmi_fence();
  }
  return success;
}
bool many_pushbacks_onto_size_one()
{
  int vTotal, realTotal;
  size_t num_locs = stapl::get_num_locations();
  size_t loc = stapl::get_location_id();
  bool success = true;
  stapl::counter<stapl::default_timer> execution_timer;

  for (int n=1; n < 1000; n *= 10)
  {
    execution_timer.reset();
    for (int k=0; k < ITERATIONS; k++)
    {
      // TEST: Pushback n elements.
      vTotal = 0;
      realTotal = 0;
      vec_int_tp v(0);
      execution_timer.start();
      stapl::rmi_fence();
      for (int t= loc; t < n; t+=num_locs )
      {
        v.push_back(t);
      }
      stapl::rmi_fence();
      execution_timer.stop();

      // Validate Test
      for (int t= 0; t < n; t++ )
      {
      vTotal += v[t]; // We start with a size-one vector.
      realTotal += t;
      }
      stapl::rmi_fence();

      if (vTotal != realTotal)
      success = false;
    }
    // Output
    if (loc == 0  && VERBOSE_VECTOR_TEST)
    {
      if (success)
      cout << "PASS: many_pushbacks_onto_size_zero: n=" << n << endl;
      else
      cout << "-->FAIL: many_pushbacks_onto_size_zero: n=" << n
        << ": " <<  vTotal << " != " << realTotal << endl;

      cout << "  execution_timer.value() (" << ITERATIONS << "): "
        << execution_timer.value() << " seconds." << endl;
    }
    stapl::rmi_fence();
  }
  return success;
}
