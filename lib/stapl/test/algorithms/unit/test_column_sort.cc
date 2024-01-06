/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/array.hpp>
#include <stapl/algorithms/sorting.hpp>
#include <stapl/algorithms/column_sort.hpp>

#include <stdlib.h>
#include <time.h>

#include <iomanip>

#include "../../confint.hpp"

#define DEBUG false

//////////////////////////////////////////////////////////////////////
/// @brief Prints a 1d view
/// @tparam View The view to print.
/// @param title view name.
//////////////////////////////////////////////////////////////////////
template <typename View>
void print_1d_array(View&& view, std::string title)
{
  stapl::do_once([&] {
    std::cout << title << std::endl;

    size_t size =view.domain().dimensions();

    for (size_t i = 0; i != size; i++)
      std::cout << std::setw(4) << view.get_element(i);
    std::cout << std::endl;
  });
}


//////////////////////////////////////////////////////////////////////
/// @brief Prints a message to stdout.
/// @param s the string to print.
//////////////////////////////////////////////////////////////////////
void message(std::string s)
{
  stapl::do_once([&]() { std::cout << s; });
}


//////////////////////////////////////////////////////////////////////
/// @brief Initialize data using Leighton's paper example
/// @tparam View The view to fill
//////////////////////////////////////////////////////////////////////
template <typename View>
void initialize_test_data(View& view)
{
  size_t idx = 0;
  view.set_element(idx++, 6);
  view.set_element(idx++, 14);
  view.set_element(idx++, 10);
  view.set_element(idx++, 3);
  view.set_element(idx++, 17);
  view.set_element(idx++, 5);
  view.set_element(idx++, 15);
  view.set_element(idx++, 4);
  view.set_element(idx++, 1);
  view.set_element(idx++, 16);
  view.set_element(idx++, 8);
  view.set_element(idx++, 11);
  view.set_element(idx++, 12);
  view.set_element(idx++, 7);
  view.set_element(idx++, 13);
  view.set_element(idx++, 9);
  view.set_element(idx++, 2);
  view.set_element(idx++, 0);
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function to fill input view with random values within
/// a known range that doesn't include the infinity values for column_sort
//////////////////////////////////////////////////////////////////////
template <typename T>
struct fill_random
{
private:
  typedef std::uniform_int_distribution<T> distrib_type;

  std::mt19937 m_gen;
  distrib_type m_dist;

public:
  fill_random(unsigned int seed)
    : m_gen(seed), m_dist(-1000000, 1000000)
  { }

  template <typename Element>
  void operator() (Element&& element)
  {
    element = m_dist(m_gen);
  }

  void define_type(stapl::typer& t)
  {
    stapl::abort("Serializing fill_random. std::mt19937 not serializable");
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Initialize data using random numbers
/// @tparam View The view to fill
//////////////////////////////////////////////////////////////////////
template <typename View>
void initialize_random_data(View& view, unsigned int seed)
{
  stapl::map_func(fill_random<typename View::value_type>(seed), view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Test ColumnSort implementation
//////////////////////////////////////////////////////////////////////
stapl::exit_code stapl_main(int argc, char *argv[])
{
  if (argc < 2)
  {
    message("usage: test_column_sort <size> [samples]\n");
    exit(0);
  }

  size_t size    = atol(argv[1]);
  int    samples = 1;

  if (argc > 2)
    samples = atoi(argv[2]);

  auto seed = time(NULL);

  typedef stapl::array<long> test_array_t;
  typedef stapl::array_view<test_array_t> test_array_view_t;

  test_array_t test1_array(size);
  test_array_t test2_array(size);

  test_array_view_t test1_array_view(test1_array);
  test_array_view_t test2_array_view(test2_array);

  std::vector<double> samples0(samples, 0.);
  std::vector<double> samples1(samples, 0.);

  stapl::counter<stapl::default_timer> timer;

  for (int i = 0; i != samples; ++i)
  {
    if (size == 18) {
      initialize_test_data(test1_array_view);
      initialize_test_data(test2_array_view);
    } else {
      initialize_random_data(test1_array_view, seed);
      initialize_random_data(test2_array_view, seed);
    }


    if (DEBUG)
      print_1d_array(test1_array_view, "--- test 1 - original ---");

    timer.reset();
    timer.start();

    stapl::column_sort(test1_array_view);

    samples0[i] = timer.stop();

    if (DEBUG)
      print_1d_array(test1_array_view, "--- test 1 - sorted ---");

    if (DEBUG)
      print_1d_array(test2_array_view, "\n--- test 2 - original ---");

    timer.reset();
    timer.start();

    stapl::column_sort(test2_array_view, stapl::greater<long>());

    samples1[i] = timer.stop();

    if (DEBUG)
      print_1d_array(test2_array_view, "--- test 2 - sorted ---");
  }

  bool test1 = stapl::is_sorted(test1_array_view);
  bool test2 = stapl::is_sorted(test2_array_view, std::greater<long>());

  auto stats0 = compute_stats(samples0);
  auto stats1 = compute_stats(samples1);

  std::stringstream ss;

  ss << "Test: column_sort\n";
  if (test1)
    ss << "Status: PASS\n";
  else
    ss << "Status: FAIL\n";
  ss << "Version: stapl\n";
  ss << "Time: " << stats0.avg << "\n";
  if (samples != 1)
    ss << "Notes: (ci, min, max, stddev, num samples) "
       << stats0.conf_interval << " " << stats0.min << " " << stats0.max << " "
       << stats0.stddev << " " << stats0.num_samples << "\n\n";
  else
    ss << "\n";

  ss << "Test: column_sort_with_comparator\n";
  if (test2)
    ss << "Status: PASS\n";
  else
    ss << "Status: FAIL\n";
  ss << "Version: stapl\n";
  ss << "Time: " << stats1.avg << "\n";
  if (samples != 1)
    ss << "Notes: (ci, min, max, stddev, num samples) "
       << stats1.conf_interval << " " << stats1.min << " " << stats1.max << " "
       << stats1.stddev << " " << stats1.num_samples << "\n\n";
  else
    ss << "\n";

  message(ss.str());

  if (test1 && test2)
    return EXIT_SUCCESS;
  else
    return EXIT_FAILURE;
}
