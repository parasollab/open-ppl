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
/// Test zero-copy support in runtime.
///
/// This file also tests if combining is performed correctly.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <stapl/runtime/type_traits/is_movable.hpp>
#include "test_utils.h"

using namespace stapl;

// wrapper around T with move constructor and move assignment enabled
template<typename T>
class wrapper
{
private:
  T   m_t;
  int m_copied;

public:
  wrapper(T const& t)
  : m_t(t),
    m_copied(0)
  { }

  wrapper(wrapper const& other)
  : m_t(other.m_t),
    m_copied(other.m_copied + 1)
  { }

  wrapper(wrapper&& other)
  : m_t(std::move(other.m_t)),
    m_copied(other.m_copied)
  { other.m_copied = 0; }

  wrapper& operator=(wrapper const&) = delete;

  T const& get(void) const noexcept
  { return m_t; }

  T& get(void) noexcept
  { return m_t; }

  int num_copied(void) const noexcept
  { return m_copied; }

  void define_type(typer& t)
  {
    t.member(m_t);
    t.member(m_copied);
  }
};

static_assert(runtime::is_movable<wrapper<std::vector<int>>>::value,
              "wrapper<T> is not movable");


// wrapper around T with move constructor and move assignment enabled
template<typename T>
class shared_ptr_wrapper_no_move
{
private:
  std::shared_ptr<T> m_p;

public:
  shared_ptr_wrapper_no_move(void) = default;

  shared_ptr_wrapper_no_move(T const& t)
  : m_p(std::make_shared<T>(t))
  { }

  shared_ptr_wrapper_no_move(shared_ptr_wrapper_no_move const&) = default;
  shared_ptr_wrapper_no_move(shared_ptr_wrapper_no_move&&) = delete;

  long use_count(void) const
  { return m_p.use_count(); }

  void define_type(typer& t)
  { t.member(m_p); }
};

static_assert(!runtime::is_movable<shared_ptr_wrapper_no_move<int>>::value,
              "shared_ptr_wrapper_no_move<T> is movable");


// wrapper around shared_ptr_wrapper_no_move that allows moving
template<typename T>
class shared_ptr_wrapper_wrapper
{
private:
  shared_ptr_wrapper_no_move<T> m_p;

public:
  shared_ptr_wrapper_wrapper(void) = default;

  shared_ptr_wrapper_wrapper(T const& t)
  : m_p(t)
  { }

  long use_count(void) const
  { return m_p.use_count(); }

  void define_type(typer& t)
  { t.member(m_p); }
};

static_assert(runtime::is_movable<shared_ptr_wrapper_wrapper<int>>::value,
              "shared_ptr_wrapper_wrapper<T> is movable");


struct p_test
: public p_test_object
{
  p_test(void)
  { this->advance_epoch(); }

  template<typename T>
  void receiver(T)
  { }

  template<typename T>
  void receive(T t, const int copied)
  { STAPL_RUNTIME_TEST_CHECK(copied, t.num_copied()); }

  template<typename T>
  void receive_ref(T& t, const int copied)
  { STAPL_RUNTIME_TEST_CHECK(copied, t.num_copied()); }

  template<typename T>
  void receive_cref(T const& t, const int copied)
  { STAPL_RUNTIME_TEST_CHECK(copied, t.num_copied()); }

  template<typename T>
  void receive_move(T&& t, const int copied)
  { STAPL_RUNTIME_TEST_CHECK(copied, t.num_copied()); }

  template<typename T>
  void receive_shared_ptr_wrapper(T const& t)
  { STAPL_RUNTIME_TEST_CHECK(t.use_count(), 1); }

  // local test of support classes
  void test_local_int(void)
  {
    typedef int                 value_type;
    typedef wrapper<value_type> wrapper_type;

    STAPL_RUNTIME_TEST_CHECK(
      runtime::is_movable<value_type>::value, false
    );
    STAPL_RUNTIME_TEST_CHECK(
      runtime::is_movable<wrapper_type>::value, true
    );

    value_type t = 42;

    wrapper_type w1{t};
    receive<wrapper_type>(w1, 1);
    receive<wrapper_type>(std::move(w1), 0);

    wrapper_type w2{t};
    receive_ref<wrapper_type>(w2, 0);
    receive_cref<wrapper_type>(w2, 0);
    receive_move<wrapper_type>(std::move(w2), 0);
  }

  // local test of support classes
  void test_local_vector_int(void)
  {
    typedef std::vector<int>    value_type;
    typedef wrapper<value_type> wrapper_type;

    STAPL_RUNTIME_TEST_CHECK(
      runtime::is_movable<value_type>::value, true
    );
    STAPL_RUNTIME_TEST_CHECK(
      runtime::is_movable<wrapper_type>::value, true
    );

    value_type t = { 0, 1, 2, 3 };

    wrapper_type w1{t};
    receive<wrapper_type>(w1, 1);
    receive<wrapper_type>(std::move(w1), 0);

    wrapper_type w2{t};
    receive_ref<wrapper_type>(w2, 0);
    receive_cref<wrapper_type>(w2, 0);
    receive_move<wrapper_type>(std::move(w2), 0);
  }

  // test zero-copy of wrapper<int>
  void test_int(void)
  {
    typedef int                 value_type;
    typedef wrapper<value_type> wrapper_type;

    value_type t = 42;

    wrapper_type w1{t};
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive<wrapper_type>, w1, 1);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive<wrapper_type>, std::move(w1), 0);

    wrapper_type w2{t};
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_ref<wrapper_type>, w2, 1);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_ref<wrapper_type>, std::move(w2), 0);

    wrapper_type w3{t};
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_cref<wrapper_type>, w3, 0);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_cref<wrapper_type>, std::move(w3), 0);

    wrapper_type w4{t};
#if 0
    // this should not compile
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_move<wrapper_type>, w4, 0);
#endif
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_move<wrapper_type>, std::move(w4), 0);

    rmi_fence(); // quiescence before next test
  }

  // test zero-copy of wrapper<std::vector<int>>
  void test_vector_int(void)
  {
    typedef std::vector<int>    value_type;
    typedef wrapper<value_type> wrapper_type;

    value_type t = {1, 2, 3, 4};

    wrapper_type w1{t};
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive<wrapper_type>, w1, 1);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive<wrapper_type>, std::move(w1), 0);
    STAPL_RUNTIME_TEST_CHECK(w1.get().size(), 0);

    wrapper_type w2{t};
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_ref<wrapper_type>, w2, 1);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_ref<wrapper_type>, std::move(w2), 0);
    STAPL_RUNTIME_TEST_CHECK(w2.get().size(), 0);

    wrapper_type w3{t};
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_cref<wrapper_type>, w3, 0);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_cref<wrapper_type>, std::move(w3), 0);
    STAPL_RUNTIME_TEST_CHECK(w2.get().size(), 0);

    wrapper_type w4{t};
#if 0
    // this should not compile
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_move<wrapper_type>, w4, 0);
#endif
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_move<wrapper_type>, std::move(w4), 0);

    rmi_fence(); // quiescence before next test
  }

  // test variations on shared_ptr
  void test_shared_ptr(void)
  {
    auto p1 = std::make_shared<int>(42);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_shared_ptr_wrapper<decltype(p1)>, p1);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_shared_ptr_wrapper<decltype(p1)>, std::move(p1));

    shared_ptr_wrapper_no_move<int> p2(42);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_shared_ptr_wrapper<decltype(p2)>, p2);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_shared_ptr_wrapper<decltype(p2)>, std::move(p2));

    shared_ptr_wrapper_wrapper<int> p3(42);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_shared_ptr_wrapper<decltype(p3)>, p3);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::receive_shared_ptr_wrapper<decltype(p3)>, std::move(p3));

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_local_int();
    test_local_vector_int();
    test_int();
    test_vector_int();
    test_shared_ptr();
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
