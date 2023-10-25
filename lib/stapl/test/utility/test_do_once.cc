/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/utility/do_once.hpp>
#include <stapl/runtime.hpp>

#include <iostream>
#include <boost/bind.hpp>

void foo() {
  std::cout << __func__ << "() called" << std::endl;
}

void foo1() {
  std::cout << __func__ << "() called" << std::endl;
}

void foo1(int i) {
  std::cout << __func__ << "() called: " << i << std::endl;
}

void foo1(int i, int j) {
  std::cout << __func__ << "() called: " << i << "," << j << std::endl;
}

int foo_i() {
  return 42;
}

struct my_functor1 {
  void operator()() {
    std::cout << "my_functor1::" << __func__ << "() called" << std::endl;
  }
};

struct my_functor2 {
  int operator()() {
    return 27;
  }
};

struct my_functor3 {
  typedef int result_type;
  int operator()() {
    return 27;
  }
};


void test_functions() {
  stapl::do_once( foo );

  stapl::do_once( foo1 );

  stapl::do_once( foo1, 42 );

  stapl::do_once( foo1, 42, 42 );

  stapl::do_once(foo_i);

  int r = stapl::do_once(foo_i);
  std::cout << "Node " << stapl::get_location_id() << " r = " << r << std::endl;
  stapl::rmi_fence();
}



void test_function_objects() {
  using boost::bind;

  stapl::do_once( (bind<void>(foo1, 10, 20)) );

  stapl::do_once( (bind<void>(foo1, 10, 20)) );

  stapl::do_once( (bind<void>(foo1, _1, 20)), 9 );

  // somehow this should work - result_type is defined
  int k = stapl::do_once( (bind<int>(foo_i)) );
  std::cout << "Node " << stapl::get_location_id() << " k = " << k << std::endl;

  stapl::do_once( my_functor1() );
  int l = stapl::do_once( my_functor2() );
  std::cout << "Node " << stapl::get_location_id() << " l = " << l << std::endl;

  int m = stapl::do_once( my_functor3() );
  std::cout << "Node " << stapl::get_location_id() << " m = " << m << std::endl;

  bool r = true;
  stapl::do_once([=](void) {
    if (r == true)
      std::cout << "True...\n";
    else
      std::cout << "False...\n";
  });
}


stapl::exit_code stapl_main(int, char*[])
{
  test_functions();
  test_function_objects();
  return EXIT_SUCCESS;
}
