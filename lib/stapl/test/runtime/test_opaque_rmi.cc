/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Test @ref stapl::opaque_rmi().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include "test_utils.h"

using namespace stapl;

struct p_test
: public p_object
{
  unsigned int left, right; // neighbor id's

  p_test(void)
  {
    const unsigned int id = this->get_location_id();
    right = (id == this->get_num_locations() - 1) ? 0 : id + 1;
    left = (id == 0) ? this->get_num_locations() - 1 : id - 1;
    this->advance_epoch();
  }

  void fV(void)
  { }

  bool fB(bool t)
  { return t; }

  char fC(char t)
  { return t; }

  signed char fSC(signed char t)
  { return t; }

  unsigned char fUC(unsigned char t)
  { return t; }

  signed short int fSSI(signed short int t)
  { return t; }

  unsigned short int fUSI(unsigned short int t)
  { return t; }

  signed int fSI(signed int t)
  { return t; }

  unsigned int fUI(unsigned int t)
  { return t; }

  signed long fSL(signed long t)
  { return t; }

  unsigned long fUL(unsigned long t)
  { return t; }

  float fF(float t)
  { return t; }

  double fD(double t)
  { return t; }

  long double fLD(long double t)
  { return t; }

  // Tests opaque_rmi()
  void test_fundamental1(void)
  {
    future<void> hV = opaque_rmi(right, this->get_rmi_handle(), &p_test::fV);
    hV.get();

    future<bool> hB = opaque_rmi(right, this->get_rmi_handle(),
                                 &p_test::fB, false);
    STAPL_RUNTIME_TEST_CHECK(false, hB.get());

    future<char> hC1 = opaque_rmi(right, this->get_rmi_handle(),
                                  &p_test::fC, '1');
    STAPL_RUNTIME_TEST_CHECK('1', hC1.get());

    future<char> hC2 = opaque_rmi(right, this->get_rmi_handle(),
                                  &p_test::fC, (signed char)'2');
    STAPL_RUNTIME_TEST_CHECK('2', hC2.get());

    future<char> hC3 = opaque_rmi(right, this->get_rmi_handle(),
                                  &p_test::fC, (unsigned char)'3');
    STAPL_RUNTIME_TEST_CHECK('3', hC3.get());

    future<char> hC4 = opaque_rmi(0, this->get_rmi_handle(),
                                  &p_test::fC, (unsigned char)'a');
    STAPL_RUNTIME_TEST_CHECK('a', hC4.get());

    rmi_fence(); // quiescence before next test
  }

  // Tests overlapping opaque_rmi()
  void test_fundamental2(void)
  {
    future<signed short> hI1;
    future<unsigned short> hI2;
    future<int> hI3;
    future<unsigned int> hI4;
    future<signed long> hI5;
    future<unsigned long> hI6;
    future<float> hF1;
    future<double> hF2;
    future<long double> hF3;
    hI1 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fSSI, (signed short)4);
    hI2 = opaque_rmi(0, this->get_rmi_handle(),
                     &p_test::fUSI, (unsigned short)5);
    hI3 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fSI, 6);
    hI4 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fUI, 7U);
    hI5 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fSL, 8L);
    hI6 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fUL, 9UL);
    hF1 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fF, 1.0f);
    hF2 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fD, 1.1);
    hF3 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fLD, (long double)1.2);
    STAPL_RUNTIME_TEST_CHECK(4, hI1.get());
    STAPL_RUNTIME_TEST_CHECK(5, hI2.get());
    STAPL_RUNTIME_TEST_CHECK(6, hI3.get());
    STAPL_RUNTIME_TEST_CHECK(7, hI4.get());
    STAPL_RUNTIME_TEST_CHECK(8, hI5.get());
    STAPL_RUNTIME_TEST_CHECK(9, hI6.get());
    STAPL_RUNTIME_TEST_CHECK(
        (1.0 - hF1.get() < std::numeric_limits<float>::epsilon()), true);
    STAPL_RUNTIME_TEST_CHECK(
        (1.1 - hF2.get() < std::numeric_limits<double>::epsilon()), true);
    STAPL_RUNTIME_TEST_CHECK(
        (1.2 - hF3.get() < std::numeric_limits<long double>::epsilon()), true);

    rmi_fence(); // quiescence before next test
  }

  // Tests overlapping opaque_rmi() with async_rmi() and sync_rmi()
  void test_fundamental3(void)
  {
    future<signed short> hI1;
    future<unsigned short> hI2;
    future<int> hI3;
    future<unsigned int> hI4;
    future<signed long> hI5;
    future<unsigned long> hI6;
    future<float> hF1;
    future<double> hF2;
    future<long double> hF3;

    hI1 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fSSI, (signed short)4);
    async_rmi(0, get_rmi_handle(), &p_test::fUI, 7U);
    hI2 = opaque_rmi(0, this->get_rmi_handle(),
                     &p_test::fUSI, (unsigned short)5);
    async_rmi(0, get_rmi_handle(), &p_test::fUI, 7U);
    hI3 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fSI, 6);
    hI4 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fUI, 7U);
    sync_rmi(0, get_rmi_handle(), &p_test::fUI, 7U);
    hI5 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fSL, 8L);
    hI6 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fUL, 9UL);
    hF1 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fF, 1.0f);
    hF2 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fD, 1.1);
    hF3 = opaque_rmi(0, this->get_rmi_handle(), &p_test::fLD, (long double)1.2);

    STAPL_RUNTIME_TEST_CHECK(4, hI1.get());
    STAPL_RUNTIME_TEST_CHECK(5, hI2.get());
    STAPL_RUNTIME_TEST_CHECK(6, hI3.get());
    STAPL_RUNTIME_TEST_CHECK(7, hI4.get());
    STAPL_RUNTIME_TEST_CHECK(8, hI5.get());
    STAPL_RUNTIME_TEST_CHECK(9, hI6.get());
    STAPL_RUNTIME_TEST_CHECK(
        (1.0 - hF1.get() < std::numeric_limits<float>::epsilon()), true);
    STAPL_RUNTIME_TEST_CHECK(
        (1.1 - hF2.get() < std::numeric_limits<double>::epsilon()), true);
    STAPL_RUNTIME_TEST_CHECK(
        (1.2 - hF3.get() < std::numeric_limits<long double>::epsilon()), true);

    rmi_fence(); // quiescence before next test
  }

  // Tests an arbitrary object with no pointers
  stack_object stack_object_return(stack_object const& t)
  {
    stack_object tt = t;
    STAPL_RUNTIME_TEST_CHECK(tt, stack_object(right));
    return stack_object(right + 1);
  }

  void test_stack_object(void)
  {
    stack_object o0(this->get_location_id());
    stack_object o1(this->get_location_id() + 1);
    future<stack_object> h = opaque_rmi(left, this->get_rmi_handle(),
                                        &p_test::stack_object_return, o0);

    STAPL_RUNTIME_TEST_CHECK(h.get(), o1);
    rmi_fence(); // quiescence before next test
  }

  // Tests an arbitrary object with pointers
  dynamic_object dynamic_object_return(dynamic_object const& t)
  {
    dynamic_object tt = t;
    STAPL_RUNTIME_TEST_CHECK(tt, dynamic_object(left));
    return dynamic_object(left + 1);
  }

  void test_dynamic_object(void)
  {
    dynamic_object o0(this->get_location_id());
    dynamic_object o1(this->get_location_id() + 1);
    future<dynamic_object> h = opaque_rmi(right, this->get_rmi_handle(),
                                          &p_test::dynamic_object_return, o0);

    STAPL_RUNTIME_TEST_CHECK(h.get(), o1);
    rmi_fence(); // quiescence before next test
  }

  // Tests an std::vector<int> without overlapping
  std::vector<int> vector_return(std::size_t s)
  {
    std::vector<int> v;
    make_vector(v, s);
    STAPL_RUNTIME_TEST_CHECK(check_vector(v, s), true);
    return v;
  }

  void test_vector(std::vector<int>& v, const std::size_t s)
  {
    STAPL_RUNTIME_TEST_CHECK(check_vector(v, s), true);
  }

  void make_vector(std::vector<int>& v, const std::size_t size)
  {
    for (int i = size - 1; i >= 0; --i)
      v.push_back(i);
  }

  bool check_vector(std::vector<int>& v, const std::size_t size)
  {
    if (v.size() != size)
      return false;
    for (int start = size - 1; start >= 0; start--) {
      if (v[size - start - 1] != start)
        return false;
    }
    return true;
  }

  void test_vector1(void)
  {
    const std::size_t N = 100;
    for (std::size_t i = N / 100; i < N; ++i) {
      future<std::vector<int> > h = opaque_rmi(right, this->get_rmi_handle(),
                                               &p_test::vector_return, i);
      std::vector<int> v = h.get();
      test_vector(v, i);
    }

    rmi_fence(); // quiescence before next test
  }

  // Tests an std::vector<int> with overlapping
  void test_vector2(void)
  {
    const std::size_t N = 100;

    future<std::vector<int> > h[N];
    for (std::size_t i = 0; i < N; ++i) {
      h[i] = opaque_rmi(right, this->get_rmi_handle(),
                        &p_test::vector_return, N + i * N);
    }

    for (std::size_t i = 0; i < N; ++i) {
      h[i].wait();
    }

    for (std::size_t i = 0; i < N; ++i) {
      std::vector<int> v = h[i].get();
      test_vector(v, N + i * N);
    }

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_fundamental1();
    test_fundamental2();
    test_fundamental3();
    test_stack_object();
    test_dynamic_object();
    test_vector1();
    test_vector2();
  }
};


exit_code stapl_main(int, char*[])
{
  p_test pt;
  pt.execute();

#ifndef _TEST_QUIET
  std::cout << get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}
