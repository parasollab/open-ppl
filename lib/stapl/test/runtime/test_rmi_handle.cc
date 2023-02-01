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
/// Test @ref stapl::rmi_handle transport.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <algorithm>
#include <random>
#include <iostream>
#include <memory>
#include <vector>
#include "test_utils.h"

using namespace stapl;

// base p_object
class base_object
{
private:
  rmi_handle m_handle;

public:
  base_object(void)
  : m_handle{this}
  { }
};

// derived p_object that is also registered
class derived_object
: public base_object
{
private:
  rmi_handle m_handle;

public:
  derived_object(void)
  : m_handle{this}
  { }
};

// twice registered p_object; should trigger an error
class multiregistered_object
{
private:
  rmi_handle m_first_handle;
  rmi_handle m_second_handle;

public:
  multiregistered_object(void)
  : m_first_handle{this},
    m_second_handle{this}
  { }
};

class simple_object
{
private:
  int        m_i;
  rmi_handle m_handle;

public:
  explicit simple_object(int i = -1)
  : m_i{i},
    m_handle{this}
  { }

  rmi_handle::const_reference const& get_rmi_handle(void) const
  { return m_handle; }

  rmi_handle::reference const& get_rmi_handle(void)
  { return m_handle; }

  void test(int i) const
  { STAPL_RUNTIME_TEST_CHECK(m_i, i); }

  void set(int i)
  { m_i = i; }

  int get(void) const
  { return m_i; }

  friend bool operator==(simple_object const& x,
                         simple_object const& y) noexcept
  { return (x.m_i==y.m_i); }

  friend bool operator!=(simple_object const& x,
                         simple_object const& y) noexcept
  { return !(x==y); }
};

class p_test
: public p_object
{
private:
  unsigned int m_lt; // left neighbor id
  unsigned int m_rt; // right neighbor id

public:
  p_test(void)
  : m_lt{},
    m_rt{}
  {
    const auto my_id = this->get_location_id();
    m_lt = (my_id==0) ? (this->get_num_locations() - 1) : (my_id - 1);
    m_rt = (my_id==this->get_num_locations() - 1) ? 0 : (my_id + 1);
    this->advance_epoch();
  }

  // tests registration of base and derived classes
  void test_derived(void)
  {
    base_object b;
    derived_object d;
  }

  // tests multiple registration of the same object; not allowed
  void test_multi_registration(void)
  {
    multiregistered_object o;
  }

  // tests registration and unregistration
  void test_in_order(void)
  {
    using simple_object_ptr = std::unique_ptr<simple_object>;

    const int N = 50;

    // construct p_objects in-order
    std::vector<simple_object_ptr> v;
    for (int i=0; i<N; ++i) {
      v.push_back(simple_object_ptr{new simple_object(i)});
    }

    // verify that all handles are what are supposed to be
    for (int i=0; i<N; ++i) {
      unordered::async_rmi(all_locations, v[i]->get_rmi_handle(),
                           &simple_object::test, i);
    }
    rmi_fence(); // wait for async_rmi calls to finish

    // delete objects in-order
    for (int i=0; i<N; ++i) {
      v[i].reset();
    }

    rmi_fence(); // quiescence before next test
  }

  // tests registration and out-of-order unregistration
  void test_out_of_order(int k = 5)
  {
    using simple_object_ptr = std::unique_ptr<simple_object>;

    const int N = 50;

    if (k==0) return;

    // construct p_objects
    std::vector<simple_object_ptr> v;
    for (int i=0; i<N; ++i) {
      v.emplace_back(simple_object_ptr{new simple_object(i)});
    }

    // verify that all handles are what are supposed to be
    for (int i=0; i<N; ++i) {
      unordered::async_rmi(all_locations, v[i]->get_rmi_handle(),
                           &simple_object::test, i);
    }
    rmi_fence(); // wait for async_rmi calls to finish before checking

    // delete objects in random order
    std::mt19937 g(this->get_location_id() * k);
    std::shuffle(v.begin(), v.end(), g);
    for (int i=0; i<N; ++i) {
      v[i].reset();
    }

    // do it again to see if handles are correct
    test_out_of_order(k-1);

    rmi_fence(); // quiescence before next test
  }

  void callback(rmi_handle::reference rh, const int n)
  { async_rmi(m_rt, rh, &simple_object::set, n); }

  void callback_const(rmi_handle::const_reference rh, const int n) const
  { async_rmi(m_rt, rh, &simple_object::test, n); }

  // tests rmi_handle packing and rmi_handle references
  void test_rmi_handle_packing(void)
  {
    const auto lid = this->get_location_id();
    simple_object o;
    rmi_handle::reference ref        = o.get_rmi_handle();
    rmi_handle::const_reference cref = ref;
    async_rmi(m_lt, this->get_rmi_handle(), &p_test::callback, ref, lid);
    async_rmi(m_lt, this->get_rmi_handle(), &p_test::callback_const, cref, lid);

    rmi_fence(); // wait for async_rmi calls to finish before checking
    STAPL_RUNTIME_TEST_CHECK(o.get(), int(lid));

    rmi_fence(); // quiescence before next test
  }

  // tests resolve_handle and get_p_object
  void test_retrieval(void)
  {
    simple_object i;
    auto& h_i = i.get_rmi_handle();

    rmi_handle::reference ref_i = h_i;
    STAPL_RUNTIME_TEST_CHECK(&i, resolve_handle<simple_object>(ref_i));
    STAPL_RUNTIME_TEST_CHECK(i, get_p_object<simple_object>(ref_i));

    rmi_handle::const_reference cref_i = h_i;
    STAPL_RUNTIME_TEST_CHECK(&i, resolve_handle<simple_object>(cref_i));
    STAPL_RUNTIME_TEST_CHECK(i, get_p_object<simple_object>(cref_i));

    rmi_handle::light_reference lref_i = h_i;
    STAPL_RUNTIME_TEST_CHECK(&i, resolve_handle<simple_object>(lref_i));
    STAPL_RUNTIME_TEST_CHECK(i, get_p_object<simple_object>(lref_i));

    rmi_handle::const_light_reference clref_i = h_i;
    STAPL_RUNTIME_TEST_CHECK(&i, resolve_handle<simple_object>(clref_i));
    STAPL_RUNTIME_TEST_CHECK(i, get_p_object<simple_object>(clref_i));
  }

  // tests rmi_handle references
  void test_equality(void)
  {
    simple_object i, j;
    auto& h_i = i.get_rmi_handle();
    auto& h_j = j.get_rmi_handle();

    // create references
    rmi_handle::reference ref_i = h_i;
    rmi_handle::const_reference cref_i = h_i;
    rmi_handle::light_reference lref_i = h_i;
    rmi_handle::const_light_reference clref_i = h_i;

    // check rmi_handle::reference
    STAPL_RUNTIME_TEST_CHECK(h_i, ref_i);
    STAPL_RUNTIME_TEST_CHECK(ref_i, h_i);
    STAPL_RUNTIME_TEST_REQUIRE(ref_i!=h_j);
    STAPL_RUNTIME_TEST_REQUIRE(h_j!=ref_i);

    // check rmi_handle::const_reference
    STAPL_RUNTIME_TEST_CHECK(h_i, cref_i);
    STAPL_RUNTIME_TEST_CHECK(cref_i, h_i);
    STAPL_RUNTIME_TEST_REQUIRE(cref_i!=h_j);
    STAPL_RUNTIME_TEST_REQUIRE(h_j!=cref_i);

    // check rmi_handle::light_reference
    STAPL_RUNTIME_TEST_CHECK(h_i, lref_i);
    STAPL_RUNTIME_TEST_CHECK(lref_i, h_i);
    STAPL_RUNTIME_TEST_REQUIRE(lref_i!=h_j);
    STAPL_RUNTIME_TEST_REQUIRE(h_j!=lref_i);

    // check rmi_handle::const_light_reference
    STAPL_RUNTIME_TEST_CHECK(h_i, clref_i);
    STAPL_RUNTIME_TEST_CHECK(clref_i, h_i);
    STAPL_RUNTIME_TEST_REQUIRE(clref_i!=h_j);
    STAPL_RUNTIME_TEST_REQUIRE(h_j!=clref_i);

    // check combinations
    STAPL_RUNTIME_TEST_CHECK(ref_i, ref_i);
    STAPL_RUNTIME_TEST_CHECK(ref_i, cref_i);
    STAPL_RUNTIME_TEST_CHECK(ref_i, lref_i);
    STAPL_RUNTIME_TEST_CHECK(ref_i, clref_i);

    STAPL_RUNTIME_TEST_CHECK(cref_i, ref_i);
    STAPL_RUNTIME_TEST_CHECK(cref_i, cref_i);
    STAPL_RUNTIME_TEST_CHECK(cref_i, lref_i);
    STAPL_RUNTIME_TEST_CHECK(cref_i, clref_i);

    STAPL_RUNTIME_TEST_CHECK(lref_i, ref_i);
    STAPL_RUNTIME_TEST_CHECK(lref_i, cref_i);
    STAPL_RUNTIME_TEST_CHECK(lref_i, lref_i);
    STAPL_RUNTIME_TEST_CHECK(lref_i, clref_i);

    STAPL_RUNTIME_TEST_CHECK(clref_i, ref_i);
    STAPL_RUNTIME_TEST_CHECK(clref_i, cref_i);
    STAPL_RUNTIME_TEST_CHECK(clref_i, lref_i);
    STAPL_RUNTIME_TEST_CHECK(clref_i, clref_i);
  }

  // tests conversions between rmi_handle references; commented out ones are not
  // allowed
  void test_conversions(void)
  {
    simple_object i;
    auto& h_i = i.get_rmi_handle();

    // create references
    rmi_handle::reference ref_i = h_i;
    rmi_handle::const_reference cref_i = h_i;
    rmi_handle::light_reference lref_i = h_i;
    rmi_handle::const_light_reference clref_i = h_i;

    {
      rmi_handle::reference r = ref_i;
      STAPL_RUNTIME_TEST_CHECK(ref_i, r);

      rmi_handle::const_reference cr = ref_i;
      STAPL_RUNTIME_TEST_CHECK(ref_i, cr);

      rmi_handle::light_reference l = ref_i;
      STAPL_RUNTIME_TEST_CHECK(ref_i, l);

      rmi_handle::const_light_reference cl = ref_i;
      STAPL_RUNTIME_TEST_CHECK(ref_i, cl);
    }

    {
#if 0
      // incorrect conversions
      rmi_handle::reference r = cref_i;
      STAPL_RUNTIME_TEST_CHECK(cref_i, r);

      rmi_handle::light_reference l = cref_i;
      STAPL_RUNTIME_TEST_CHECK(cref_i, l);
#endif
      rmi_handle::const_reference cr = cref_i;
      STAPL_RUNTIME_TEST_CHECK(cref_i, cr);

      rmi_handle::const_light_reference cl = cref_i;
      STAPL_RUNTIME_TEST_CHECK(cref_i, cl);
    }


    {
#if 0
      // incorrect conversions
      rmi_handle::reference r = lref_i;
      STAPL_RUNTIME_TEST_CHECK(lref_i, r);

      rmi_handle::const_reference cr = lref_i;
      STAPL_RUNTIME_TEST_CHECK(lref_i, cr);
#endif
      rmi_handle::light_reference l = lref_i;
      STAPL_RUNTIME_TEST_CHECK(lref_i, l);

      rmi_handle::const_light_reference cl = lref_i;
      STAPL_RUNTIME_TEST_CHECK(lref_i, cl);
    }

    {
#if 0
      // incorrect conversions
      rmi_handle::reference r = clref_i;
      STAPL_RUNTIME_TEST_CHECK(clref_i, r);

      rmi_handle::const_reference cr = clref_i;
      STAPL_RUNTIME_TEST_CHECK(clref_i, cr);

      rmi_handle::light_reference l = clref_i;
      STAPL_RUNTIME_TEST_CHECK(clref_i, l);
#endif
      rmi_handle::const_light_reference cl = clref_i;
      STAPL_RUNTIME_TEST_CHECK(clref_i, cl);
    }
  }

  void execute(void)
  {
    test_derived();
#if 0
    // should fail
    test_multi_registration();
#endif
    test_in_order();
    test_out_of_order();
    test_rmi_handle_packing();
    test_retrieval();
    test_equality();
    test_conversions();
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
