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
/// Test @ref stapl::immutable().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <algorithm>
#include <iostream>
#include <limits>
#include <unordered_map>
#include "test_utils.h"

using namespace stapl;

class A
{
private:
  int m_i;

public:
  explicit A(int i = 0)
  : m_i(i)
  { }

  void define_type(typer& t)
  { t.member(m_i); }
};

class B
{
private:
  p_object* m_p;

public:
  explicit B(p_object* p = nullptr)
  : m_p(p)
  { }

  void define_type(typer& t)
  { t.member(m_p); }
};


class p_test
: public p_object
{
public:
  p_test(void)
  { this->advance_epoch(); }

  template<typename T>
  void consume(immutable_shared<T> const& t, unsigned int n)
  {
    STAPL_RUNTIME_TEST_CHECK(t.use_count(), n);
  }

  template<typename T>
  void copy_consume(immutable_shared<T> t, unsigned int n)
  {
    STAPL_RUNTIME_TEST_CHECK(t.use_count(), n);
  }

  template<typename T>
  void move_consume(immutable_shared<T>&& t, unsigned int n)
  {
    STAPL_RUNTIME_TEST_CHECK(t.use_count(), n);
  }

  void test_no_copy(void)
  {
    auto t = make_immutable_shared<A>(10);

    sync_rmi(this->get_location_id(), this->get_rmi_handle(),
             &p_test::consume<A>, t, 2);

    sync_rmi(this->get_location_id(), this->get_rmi_handle(),
             &p_test::copy_consume<A>, t, 3);

    sync_rmi(this->get_location_id(), this->get_rmi_handle(),
             &p_test::move_consume<A>, std::move(t), 1);

    rmi_fence(); //quiescence before next test
  }

  void test_copy(void)
  {
    auto t = make_immutable_shared<B>(this);

    sync_rmi(this->get_location_id(), this->get_rmi_handle(),
             &p_test::consume<B>, t, 1);

    sync_rmi(this->get_location_id(), this->get_rmi_handle(),
             &p_test::copy_consume<B>, t, 2);

    rmi_fence(); //quiescence before next test
  }

  void test_external_serialization(void)
  {
    std::unordered_map<int, A> m;
    m.emplace(std::piecewise_construct,
              std::forward_as_tuple(0),
              std::forward_as_tuple(0));
    m.emplace(std::piecewise_construct,
              std::forward_as_tuple(1),
              std::forward_as_tuple(1));
    m.emplace(std::piecewise_construct,
              std::forward_as_tuple(3),
              std::forward_as_tuple(3));
    auto t = make_immutable_shared<decltype(m)>(std::move(m));

    sync_rmi(this->get_location_id(), this->get_rmi_handle(),
             &p_test::consume<decltype(m)>, t, 2);

    sync_rmi(this->get_location_id(), this->get_rmi_handle(),
             &p_test::copy_consume<decltype(m)>, t, 3);

    sync_rmi(this->get_location_id(), this->get_rmi_handle(),
             &p_test::move_consume<decltype(m)>, std::move(t), 1);

    rmi_fence(); //quiescence before next test
  }

  void execute(void)
  {
    test_no_copy();
    test_copy();
    test_external_serialization();
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
