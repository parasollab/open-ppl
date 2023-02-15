/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/array.hpp>
#include <stapl/algorithm.hpp>

using namespace stapl;


template<typename T>
void test_minmax_value(std::size_t n)
{
  using container_type = array<T>;
  using view_type      = array_view<container_type>;

  stapl::counter<stapl::default_timer> timer;
  container_type c(n);
  view_type      v(c);

  iota(v, 0);

  timer.reset();
  timer.start();
  auto mm                = minmax_value(v);
  const double exec_time = timer.stop();

  bool passed = mm.first == 0 && mm.second == static_cast<T>(n-1);

  stapl::do_once([&]()
  {
    std::cout << "Test: minmax_value(" << typeid(T).name() << "):\nStatus: ";

    if (passed)
      std::cout << "PASS" << std::endl;
    else
      std::cout << "FAIL" << std::endl;

    std::cout << "\nVersion: stapl\nTime: " << exec_time << "\n";
  });
}

template<typename T>
void test_minmax_element(std::size_t n)
{
  using container_type = array<T>;
  using view_type      = array_view<container_type>;

  container_type c(n);
  view_type      v(c);

  iota(v, 0);

  stapl::counter<stapl::default_timer> timer;
  timer.reset();
  timer.start();
  auto mm                = minmax_element(v);
  const double exec_time = timer.stop();

  bool passed = mm.first == 0 && mm.second == static_cast<T>(n-1);

  unsigned int nlocs = c.get_num_locations();

  stapl::do_once([&]()
  {
    std::cout << "Test: minmax_element(" << typeid(T).name() << "):\n";
    std::cout << "Status: ";

    if (passed)
      std::cout << "PASS" << std::endl;
    else
      std::cout << "FAIL" << std::endl;

    std::cout << "Version: stapl\n";

    std::cout << "Time: " << exec_time << "\n";

    if (nlocs == 1)
    {
      std::vector<T> c(n);
      std::iota(c.begin(), c.end(), 0);

      timer.reset();
      timer.start();
      auto res = std::minmax_element(c.begin(), c.end());
      const double stl_time = timer.stop();

      bool stl_pass = *res.first == 0 && *res.second == static_cast<T>(n-1);

      std::cout << "Status: ";

      if (stl_pass)
        std::cout << "PASS" << std::endl;
      else
        std::cout << "FAIL" << std::endl;

      std::cout << "Version: stl\n";

      std::cout << "Time: " << stl_time << "\n";
    }
  });
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2)
  {
    std::cout << "usage: " << argv[0] << " n" << std::endl;
    exit(1);
  }

  const std::size_t n = atol(argv[1]);

  test_minmax_value<int>(n);
  test_minmax_value<unsigned int>(n);
  test_minmax_value<long>(n);
  test_minmax_value<unsigned long>(n);
  test_minmax_value<float>(n);
  test_minmax_value<double>(n);

  test_minmax_element<int>(n);
  test_minmax_element<unsigned int>(n);
  test_minmax_element<long>(n);
  test_minmax_element<unsigned long>(n);
  test_minmax_element<float>(n);
  test_minmax_element<double>(n);

  return EXIT_SUCCESS;
}
